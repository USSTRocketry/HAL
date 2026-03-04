#include "WorkQueue.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace ra::hal
{
struct TaskState;

struct TaskState
{
    explicit TaskState(const WorkQueue::SubmitOptions& Options, uint64_t Sequence);

    [[nodiscard]] bool IsInfiniteTask() const;
    [[nodiscard]] bool IsFinished() const;
    [[nodiscard]] bool TryBeginExecution();
    void CompleteExecution(std::chrono::steady_clock::time_point Now);

    WorkQueue::Execution ExecutionData;
    WorkQueue::Scheduling SchedulingData;

    std::atomic<bool> CancelRequested {false};
    std::atomic<bool> Running {false};
    std::atomic<uint32_t> CompletedExecutions {0};

    uint64_t SequenceNumber = 0;
    std::atomic<std::chrono::steady_clock::duration::rep> NextRunAtTicks {0};

    std::shared_ptr<void> HandleOwner;
    std::shared_ptr<WorkQueue::WorkHandle> CallbackHandle;
};

class WorkQueue::WorkHandle::Impl
{
public:
    explicit Impl(std::shared_ptr<TaskState> InTask) : Task(std::move(InTask)) {}

    std::weak_ptr<TaskState> Task;
};

namespace
{
using QueueClock    = std::chrono::steady_clock;
using QueueTime     = QueueClock::time_point;
using QueueDuration = std::chrono::milliseconds;
using QueueTicks    = QueueClock::duration::rep;

[[nodiscard]] QueueTicks ToTicks(const QueueTime TimePoint)
{
    return TimePoint.time_since_epoch().count();
}

[[nodiscard]] QueueTime FromTicks(const QueueTicks Ticks)
{
    return QueueTime(QueueClock::duration(Ticks));
}

[[nodiscard]] bool HasHigherPriority(const std::shared_ptr<TaskState>& Left,
                                     const std::shared_ptr<TaskState>& Right)
{
    if (Left->ExecutionData.PriorityValue != Right->ExecutionData.PriorityValue)
    {
        return Left->ExecutionData.PriorityValue < Right->ExecutionData.PriorityValue;
    }

    return Left->SequenceNumber < Right->SequenceNumber;
}

void ExecuteTaskOnce(const std::shared_ptr<TaskState>& Task)
{
    if (!Task || Task->IsFinished()) { return; }
    if (!Task->TryBeginExecution()) { return; }

    if (!Task->ExecutionData.Fn || !Task->CallbackHandle)
    {
        Task->CancelRequested.store(true, std::memory_order_release);
        Task->CompleteExecution(QueueClock::now());
        return;
    }

    try
    {
        Task->ExecutionData.Fn(*Task->CallbackHandle);
    }
    catch (...)
    {
        Task->CancelRequested.store(true, std::memory_order_release);
    }

    Task->CompleteExecution(QueueClock::now());
}

void ProcessDueTasks(std::mutex& Mutex, std::vector<std::shared_ptr<TaskState>>& Tasks)
{
    const auto Now = QueueClock::now();
    std::vector<std::shared_ptr<TaskState>> ReadyTasks;

    {
        std::scoped_lock Lock(Mutex);
        ReadyTasks.reserve(Tasks.size());

        for (const auto& Task : Tasks)
        {
            if (!Task || Task->IsFinished() || Task->Running.load(std::memory_order_acquire))
            {
                continue;
            }

            const auto TaskRunAt = FromTicks(Task->NextRunAtTicks.load(std::memory_order_acquire));
            if (Now >= TaskRunAt)
            {
                ReadyTasks.push_back(Task);
            }
        }

        std::sort(ReadyTasks.begin(), ReadyTasks.end(), HasHigherPriority);
    }

    for (const auto& Task : ReadyTasks)
    {
        ExecuteTaskOnce(Task);
    }

    {
        std::scoped_lock Lock(Mutex);
        std::erase_if(Tasks,
                      [](const std::shared_ptr<TaskState>& Task)
                      {
                          return !Task
                              || (Task->IsFinished() && !Task->Running.load(std::memory_order_acquire));
                      });
    }
}
} // namespace

TaskState::TaskState(const WorkQueue::SubmitOptions& Options, const uint64_t Sequence)
    : ExecutionData(Options.Exec)
    , SchedulingData(Options.Sched)
    , SequenceNumber(Sequence)
    , NextRunAtTicks(ToTicks(QueueClock::now() + QueueDuration(Options.Sched.DelayMs)))
{
}

bool TaskState::IsInfiniteTask() const
{
    return SchedulingData.Iterations == WorkQueue::Scheduling::IterationInfinite;
}

bool TaskState::IsFinished() const
{
    if (CancelRequested.load(std::memory_order_acquire)) { return true; }
    if (IsInfiniteTask()) { return false; }
    return CompletedExecutions.load(std::memory_order_acquire) >= SchedulingData.Iterations;
}

bool TaskState::TryBeginExecution()
{
    bool Expected = false;
    return Running.compare_exchange_strong(Expected,
                                           true,
                                           std::memory_order_acq_rel,
                                           std::memory_order_acquire);
}

void TaskState::CompleteExecution(const QueueTime Now)
{
    if (!CancelRequested.load(std::memory_order_acquire) && !IsInfiniteTask())
    {
        CompletedExecutions.fetch_add(1, std::memory_order_acq_rel);
    }

    if (!IsFinished())
    {
        const auto NextRunAt = Now + QueueDuration(SchedulingData.DelayMs);
        NextRunAtTicks.store(ToTicks(NextRunAt), std::memory_order_release);
    }

    Running.store(false, std::memory_order_release);
}

class WorkQueue::Impl
{
public:
    ~Impl()
    {
        StopScheduler();
    }

    [[nodiscard]] uint64_t AllocateSequence()
    {
        return NextSequence.fetch_add(1, std::memory_order_relaxed);
    }

    void AddTask(const std::shared_ptr<TaskState>& Task)
    {
        std::scoped_lock Lock(Mutex);
        Tasks.push_back(Task);
    }

    void ClearTasks()
    {
        std::scoped_lock Lock(Mutex);
        Tasks.clear();
    }

    void NotifyScheduler()
    {
        WakeGeneration.fetch_add(1, std::memory_order_acq_rel);
        WakeSignal.notify_one();
    }

#if WORK_QUEUE_PREEMPTIVE
    void StartScheduler()
    {
        if (SchedulerThread.joinable()) { return; }

        StopRequested.store(false, std::memory_order_release);
        SchedulerThread = std::thread([this]() { SchedulerLoop(); });
    }
#endif

    void StopScheduler()
    {
        StopRequested.store(true, std::memory_order_release);
        WakeSignal.notify_all();

        if (SchedulerThread.joinable())
        {
            SchedulerThread.join();
        }
    }

    std::mutex Mutex;
    std::condition_variable WakeSignal;
    std::vector<std::shared_ptr<TaskState>> Tasks;

private:
#if WORK_QUEUE_PREEMPTIVE
    void SchedulerLoop()
    {
        while (!StopRequested.load(std::memory_order_acquire))
        {
            ProcessDueTasks(Mutex, Tasks);

            std::unique_lock Lock(Mutex);
            if (StopRequested.load(std::memory_order_acquire)) { break; }

            const auto NextWake = GetNextWakeTimeLocked();

            if (NextWake == QueueTime::max())
            {
                const auto ObservedWakeGeneration = WakeGeneration.load(std::memory_order_acquire);
                WakeSignal.wait(Lock,
                                [this, ObservedWakeGeneration]()
                                {
                                    return StopRequested.load(std::memory_order_acquire)
                                        || WakeGeneration.load(std::memory_order_acquire) != ObservedWakeGeneration
                                        || !Tasks.empty();
                                });
            }
            else
            {
                const auto ObservedWakeGeneration = WakeGeneration.load(std::memory_order_acquire);
                WakeSignal.wait_until(Lock,
                                      NextWake,
                                      [this, ObservedWakeGeneration]()
                                      {
                                          return StopRequested.load(std::memory_order_acquire)
                                              || WakeGeneration.load(std::memory_order_acquire)
                                                  != ObservedWakeGeneration;
                                      });
            }
        }
    }

    [[nodiscard]] QueueTime GetNextWakeTimeLocked() const
    {
        auto NextWake = QueueTime::max();
        for (const auto& Task : Tasks)
        {
            if (!Task || Task->IsFinished()) { continue; }
            const auto TaskRunAt = FromTicks(Task->NextRunAtTicks.load(std::memory_order_acquire));
            if (TaskRunAt < NextWake)
            {
                NextWake = TaskRunAt;
            }
        }
        return NextWake;
    }
#endif

    std::atomic<bool> StopRequested {false};
    std::atomic<uint64_t> WakeGeneration {0};
    std::thread SchedulerThread;
    std::atomic<uint64_t> NextSequence {0};
};

WorkQueue::WorkQueue() : m_Impl(std::make_unique<Impl>()) {}

WorkQueue::~WorkQueue()
{
    if (m_Impl)
    {
        m_Impl->StopScheduler();
    }
}

WorkQueue::Status WorkQueue::Init()
{
#if WORK_QUEUE_PREEMPTIVE
    m_Impl->StartScheduler();
#endif
    return Status::Success;
}

WorkQueue::Status WorkQueue::Deinit()
{
    m_Impl->StopScheduler();
    m_Impl->ClearTasks();
    return Status::Success;
}

std::pair<WorkQueue::Status, WorkQueue::WorkHandle> WorkQueue::Submit(const SubmitOptions& Options)
{
    if (!Options.Exec.Fn || Options.Sched.Iterations == 0)
    {
        return {Status::InvalidParam, WorkHandle(nullptr)};
    }

    try
    {
        auto Task = std::make_shared<TaskState>(Options, m_Impl->AllocateSequence());
        auto HandleImpl = std::make_shared<WorkHandle::Impl>(Task);

        Task->HandleOwner = HandleImpl;
        Task->CallbackHandle = std::shared_ptr<WorkHandle>(new WorkHandle(HandleImpl));

        m_Impl->AddTask(Task);
        m_Impl->NotifyScheduler();

        return {Status::Success, WorkHandle(HandleImpl)};
    }
    catch (...)
    {
        return {Status::Fail, WorkHandle(nullptr)};
    }
}

WorkQueue::Status WorkQueue::Cancel(const WorkHandle& Handle)
{
    if (const auto HandleImpl = Handle.m_Impl.lock())
    {
        if (const auto Task = HandleImpl->Task.lock())
        {
            Task->CancelRequested.store(true, std::memory_order_release);
            m_Impl->NotifyScheduler();
        }
    }

    return Status::Success;
}

#if !(WORK_QUEUE_PREEMPTIVE)
WorkQueue::Status WorkQueue::Run()
{
    ProcessDueTasks(m_Impl->Mutex, m_Impl->Tasks);
    return Status::Success;
}
#endif

WorkQueue::WorkHandle::WorkHandle(const std::shared_ptr<Impl>& HandleImpl) : m_Impl(HandleImpl) {}

WorkQueue::WorkHandle::~WorkHandle() = default;

void* WorkQueue::WorkHandle::GetContext() const
{
    const auto HandleImpl = m_Impl.lock();
    if (!HandleImpl) { return nullptr; }

    const auto Task = HandleImpl->Task.lock();
    if (!Task) { return nullptr; }

    return Task->ExecutionData.Ctx;
}

bool WorkQueue::WorkHandle::IsAlive() const
{
    const auto HandleImpl = m_Impl.lock();
    if (!HandleImpl) { return false; }

    const auto Task = HandleImpl->Task.lock();
    if (!Task) { return false; }

    return !Task->IsFinished();
}
} // namespace ra::hal