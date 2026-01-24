#pragma once

#include <cstdint>
#include <memory>
#include <utility>

class WorkQueue
{
public:
    class WorkHandle;
    using WorkFn = void(WorkHandle&);

    // Task priority
    enum class Priority : uint8_t
    {
        RealTime = 0,
        High,
        Normal,
        Low
    };

    // Execution info
    struct Execution
    {
        Priority PriorityValue = Priority::Normal;
        WorkFn* Fn             = nullptr;
        void* Ctx              = nullptr;

        // no supported yet
        WorkHandle* OldHandle = nullptr;
    };

    // Scheduler info
    struct Scheduling
    {
        uint32_t DelayMs    = 0;
        uint32_t Iterations = 1;
    };

    // Full options struct
    struct SubmitOptions
    {
        Execution Exec;
        Scheduling Sched;
    };

    enum class Status
    {
        Success,
        Fail,
        InvalidParam,
    };

public:
    std::pair<Status, WorkHandle> Submit(const SubmitOptions& Options);
    Status Cancel(const WorkHandle& Handle);

    Status Init();
    Status Deinit();

#if !(WORK_QUEUE_PREEMPTIVE)
    Status Run();
#endif

public:
    WorkQueue();
    ~WorkQueue();

    // Non-copyable, movable
    WorkQueue(const WorkQueue&)            = delete;
    WorkQueue& operator=(const WorkQueue&) = delete;
    WorkQueue(WorkQueue&&)                 = default;
    WorkQueue& operator=(WorkQueue&&)      = default;

private:
    class Impl;
    std::unique_ptr<Impl> m_Impl;
};

class WorkQueue::WorkHandle
{
public:
    void* GetContext() const;
    bool IsAlive() const;

public:
    WorkHandle(const WorkHandle&)            = default;
    WorkHandle& operator=(const WorkHandle&) = default;
    WorkHandle(WorkHandle&&)                 = default;
    WorkHandle& operator=(WorkHandle&&)      = default;

    ~WorkHandle();

private:
    // only queue implementation can manipulate
    class Impl;
    std::weak_ptr<Impl> m_Impl;

    friend class WorkQueue;

private:
    // Only WorkQueue can create this obj
    explicit WorkHandle(const std::shared_ptr<class Impl>&);
};
