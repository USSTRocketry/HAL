#include "WorkQueue.h"
#include <cassert>

// teensy41 only has a single core, so no mutex needed for now ...
// #include <mutex>

#define _TASK_PRIORITY
#define _TASK_SLEEP_ON_IDLE_RUN
#define _TASK_STD_FUNCTION
#define _TASK_SELF_DESTRUCT
#include <TaskScheduler.h>

using Wq = WorkQueue;
using Wh = Wq::WorkHandle;

////////////////////////////////////
////////// Work Handle /////////////
////////////////////////////////////

// Lightweight class that can be easily copied around
class Wh::Impl
{
public:
    Impl(Task* InTask, void* InCtx) : Work(InTask), Ctx(InCtx) {}

public:
    Task* Work = nullptr;
    void* Ctx  = nullptr;
};

Wh::WorkHandle(const std::shared_ptr<Wh::Impl>& Handle) : m_Impl(Handle) {}

bool Wh::IsAlive() const { return !m_Impl.expired(); }

void* Wh::GetContext() const
{
    auto Handle = m_Impl.lock();
    if (!Handle || !Handle->Work) { return nullptr; }

    return Handle->Ctx;
}

Wh::~WorkHandle() = default;

////////////////////////////////////
////////// Work Queue //////////////
////////////////////////////////////

// Impl definition
class Wq::Impl
{
public:
    Scheduler SchedulerRealTime;
    Scheduler SchedulerHigh;
    Scheduler SchedulerNormal;
    Scheduler SchedulerLow;

    Impl()
    {
        // Setup priority chain: RealTime -> High -> Normal -> Low
        SchedulerHigh.setHighPriorityScheduler(&SchedulerRealTime);
        SchedulerNormal.setHighPriorityScheduler(&SchedulerHigh);
        SchedulerLow.setHighPriorityScheduler(&SchedulerNormal);
    }
};

// Constructor
WorkQueue::WorkQueue() : m_Impl(std::make_unique<WorkQueue::Impl>()) {}

WorkQueue::~WorkQueue() = default;

// Initialize scheduler
WorkQueue::Status WorkQueue::Init() { return Status::Success; }

// Deinitialize scheduler
WorkQueue::Status WorkQueue::Deinit() { return Status::Success; }

// Submit a task
std::pair<Wq::Status, Wh> Wq::Submit(const SubmitOptions& Options)
{
    if (!Options.Exec.Fn || Options.Sched.Iterations == 0) return {Status::InvalidParam, Wh {nullptr}};

    // Select Scheduler based on priority
    Scheduler& TargetScheduler = [&]() -> Scheduler&
    {
        switch (Options.Exec.PriorityValue)
        {
            case Priority::RealTime:
                return m_Impl->SchedulerRealTime;
            case Priority::High:
                return m_Impl->SchedulerHigh;
            case Priority::Normal:
                return m_Impl->SchedulerNormal;
            case Priority::Low:
                return m_Impl->SchedulerLow;
            default:
                assert(false);
                return m_Impl->SchedulerNormal;
        }
    }();

    // Create Task on the heap
    auto* TaskPtr = new Task(static_cast<unsigned long>(Options.Sched.DelayMs),
                             static_cast<long>(Options.Sched.Iterations),
                             nullptr,
                             &TargetScheduler,
                             true,
                             nullptr,
                             nullptr,
                             // self destruct also frees the memory
                             true);

    auto HandleImpl = std::make_shared<Wh::Impl>(TaskPtr, Options.Exec.Ctx);
    Wh Handle {HandleImpl};

    // Callback captures a copy of WorkHandle safely
    TaskPtr->setCallback([Fn = Options.Exec.Fn, Handle]() mutable -> void { Fn(Handle); });
    TaskPtr->setOnDisable(
        [HandleImplCopy = HandleImpl]() // capture by copy to extend lifetime until Task deletion
        {
            // Nothing needed inside; destruction of this lambda releases the shared_ptr,
            // and only the Task obj holds a valid copy of this shared_ptr, all external
            // handles are holding a weak_ptr
        });

    return {Status::Success, Handle};
}

Wq::Status Wq::Cancel(const Wh& Handle)
{
    // This performs a best effort canceling,
    // Tasks already in progress might not be canceled
    auto T = Handle.m_Impl.lock();
    if (T && T->Work) { T->Work->disable(); }

    // don't really care if the task was enabled or not before this
    return Status::Success;
}

// Execute all scheduled tasks (non-preemptive)
#if !(WORK_QUEUE_PREEMPTIVE)
Wq::Status Wq::Run()
{
    // higher prior will execute first
    m_Impl->SchedulerLow.execute();
    return Status::Success;
}
#endif

/*
===============================================================================
                   WorkQueue Memory Strategy Explained
===============================================================================

Goal: Explain **why there are raw pointers, shared_ptr, and weak_ptr**, and
how memory is safely managed.

                                +-------------------+
                                |   WorkHandle      |  <-- user-facing
                                |  (copyable)       |
                                |  weak_ptr to Impl |
                                +---------+---------+
                                          |
                                          v
                                +-------------------+
                                | WorkHandle::Impl  |  <-- shared_ptr
                                |------------------|
                                | raw Task* pointer |
                                | user context ptr  |
                                +---------+---------+
                                          |
                                          v
                                +-------------------+
                                | TaskScheduler Task|
                                |   (heap-allocated)|
                                | - owns memory     |
                                | - self-destructs  |
                                +-------------------+

-------------------------------------------------------------------------------
Why Each Pointer Exists:

1. **Raw pointer (Task*) inside Impl**
   - Points directly to the TaskScheduler heap-allocated task.
   - Why raw?
     - TaskScheduler already **owns and deletes the task** via `_TASK_SELF_DESTRUCT`.
     - Shared ownership of Task* by multiple smart pointers would **duplicate ownership** and risk double-free.
   - Task* is only used internally to **set callbacks, disable tasks, or check existence**.

2. **Shared pointer (std::shared_ptr<Impl>)**
   - Impl owns the raw Task* pointer and user context.
   - Why shared?
     - The Task’s **callback captures a WorkHandle**.
     - This copy of WorkHandle captures a **shared_ptr to Impl**, guaranteeing that the callback has access to the Impl
as long as Task executes.
     - Prevents dangling pointer if the user has discarded all WorkHandles, but the Task callback is still running.

3. **Weak pointer (std::weak_ptr<Impl>) inside WorkHandle)**
   - User-facing WorkHandle holds a weak_ptr to Impl.
   - Why weak?
     - Copyable and lightweight.
     - Does not extend lifetime of Task or Impl.
     - IsAlive() and GetContext() can safely check if the Task still exists without preventing its destruction.

-------------------------------------------------------------------------------
Memory Ownership Rules / Safety Guarantees:

1. **Heap TaskPtr lifetime**
   - Managed entirely by TaskScheduler with `_TASK_SELF_DESTRUCT`.
   - No manual delete needed; prevents leaks.

2. **WorkHandle::Impl lifetime**
   - Owned by shared_ptr captured by Task callback.
   - Ensures callback can safely access Task* and context.
   - Destroyed automatically when Task callback finishes and no other shared_ptr exists.
    - Done by capturing the lifetime in the OnDisable() lambda

3. **WorkHandle lifetime**
   - Lightweight, copyable weak_ptr.
   - Can outlive Task or Impl; will correctly report IsAlive() = false once Impl/Task is gone.
   - Does NOT hold any ownership — prevents accidental retention of memory.

-------------------------------------------------------------------------------
Why this structure is correct:

- Only TaskScheduler owns Task* memory → prevents double-free.
- Shared_ptr ensures callback can safely access context and Task* while it executes → prevents dangling pointer.
- Weak_ptr in WorkHandle allows user to safely query status without extending lifetime → lightweight API.
- Copyable WorkHandle is cheap and safe → no need for mutexes on single-core MCU.

-------------------------------------------------------------------------------
Summary:

Raw pointer → points to actual Task, owned elsewhere
Shared pointer → keeps Impl alive for callback safety
Weak pointer → user-facing, copyable handle, no ownership

Assumptions:

- `_TASK_SELF_DESTRUCT` is enabled
- Single-core sequential execution
- User context remains valid for duration of task execution

===============================================================================
*/
