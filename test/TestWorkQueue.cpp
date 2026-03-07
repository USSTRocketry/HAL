#include "WorkQueue.h"

#include <atomic>
#include <chrono>
#include <gtest/gtest.h>
#include <mutex>
#include <thread>
#include <vector>

using namespace ra::hal;

namespace
{
std::atomic<int> gCountA {0};
std::atomic<int> gCountB {0};
std::mutex gOrderMutex;
std::vector<int> gOrder;

void ResetState()
{
    gCountA.store(0, std::memory_order_release);
    gCountB.store(0, std::memory_order_release);
    std::scoped_lock Lock(gOrderMutex);
    gOrder.clear();
}

void RecordOrder(const int Id)
{
    std::scoped_lock Lock(gOrderMutex);
    gOrder.push_back(Id);
}

template<typename Predicate>
bool WaitUntil(Predicate Pred, const std::chrono::milliseconds Timeout = std::chrono::milliseconds(400))
{
    const auto Deadline = std::chrono::steady_clock::now() + Timeout;
    while (std::chrono::steady_clock::now() < Deadline)
    {
        if (Pred()) { return true; }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return Pred();
}

void CountTaskA(WorkQueue::WorkHandle& Handle)
{
    (void) Handle;
    gCountA.fetch_add(1, std::memory_order_acq_rel);
}

void CountTaskB(WorkQueue::WorkHandle& Handle)
{
    (void) Handle;
    gCountB.fetch_add(1, std::memory_order_acq_rel);
}

void ContextCounterTask(WorkQueue::WorkHandle& Handle)
{
    auto* CtxCounter = static_cast<int*>(Handle.GetContext());
    if (CtxCounter)
    {
        ++(*CtxCounter);
    }
}

void OrderTask(WorkQueue::WorkHandle& Handle)
{
    auto* Id = static_cast<int*>(Handle.GetContext());
    if (Id)
    {
        RecordOrder(*Id);
    }
}

void ExpectInvalidHandle(const WorkQueue::WorkHandle& Handle)
{
    EXPECT_FALSE(Handle.IsAlive());
    EXPECT_EQ(Handle.GetContext(), nullptr);
}
} // namespace

// ============================================================================
// NON-PREEMPTIVE MODE TESTS
// ============================================================================

#if !(WORK_QUEUE_PREEMPTIVE)

TEST(MockWorkQueueApi, SubmitRejectsNullFunction)
{
    WorkQueue Queue;
    ASSERT_EQ(Queue.Init(), WorkQueue::Status::Success);

    WorkQueue::SubmitOptions Options;
    Options.Exec.Fn = nullptr;
    Options.Sched.Iterations = 1;

    auto [Status, Handle] = Queue.Submit(Options);
    EXPECT_EQ(Status, WorkQueue::Status::InvalidParam);
    ExpectInvalidHandle(Handle);
}

TEST(MockWorkQueueApi, SubmitRejectsZeroIterations)
{
    WorkQueue Queue;
    ASSERT_EQ(Queue.Init(), WorkQueue::Status::Success);

    WorkQueue::SubmitOptions Options;
    Options.Exec.Fn = &CountTaskA;
    Options.Sched.Iterations = 0;

    auto [Status, Handle] = Queue.Submit(Options);
    EXPECT_EQ(Status, WorkQueue::Status::InvalidParam);
    ExpectInvalidHandle(Handle);
}

TEST(MockWorkQueueApi, CancelOnInvalidHandleSucceeds)
{
    WorkQueue Queue;
    ASSERT_EQ(Queue.Init(), WorkQueue::Status::Success);

    WorkQueue::WorkHandle InvalidHandle;
    ExpectInvalidHandle(InvalidHandle);
    EXPECT_EQ(Queue.Cancel(InvalidHandle), WorkQueue::Status::Success);
}

TEST(MockWorkQueueApi, UserCanCreateInvalidHandle)
{
    WorkQueue::WorkHandle InvalidHandle;

    ExpectInvalidHandle(InvalidHandle);

    WorkQueue Queue;
    ASSERT_EQ(Queue.Init(), WorkQueue::Status::Success);
    EXPECT_EQ(Queue.Cancel(InvalidHandle), WorkQueue::Status::Success);
}

TEST(MockWorkQueueApi, ContextIsVisibleFromHandleAndCallback)
{
    WorkQueue Queue;
    ASSERT_EQ(Queue.Init(), WorkQueue::Status::Success);

    int CtxCounter = 0;
    WorkQueue::SubmitOptions Options;
    Options.Exec.Fn = &ContextCounterTask;
    Options.Exec.Ctx = &CtxCounter;
    Options.Sched.DelayMs = 0;
    Options.Sched.Iterations = 2;

    auto [Status, Handle] = Queue.Submit(Options);
    ASSERT_EQ(Status, WorkQueue::Status::Success);
    EXPECT_EQ(Handle.GetContext(), &CtxCounter);
    EXPECT_TRUE(Handle.IsAlive());

    EXPECT_EQ(Queue.Run(), WorkQueue::Status::Success);
    EXPECT_EQ(Queue.Run(), WorkQueue::Status::Success);
    EXPECT_EQ(CtxCounter, 2);
    EXPECT_FALSE(Handle.IsAlive());
}

TEST(MockWorkQueueApi, DelayPreventsEarlyExecution)
{
    ResetState();
    WorkQueue Queue;
    ASSERT_EQ(Queue.Init(), WorkQueue::Status::Success);

    WorkQueue::SubmitOptions Options;
    Options.Exec.Fn = &CountTaskA;
    Options.Sched.DelayMs = 30;
    Options.Sched.Iterations = 1;

    auto [Status, Handle] = Queue.Submit(Options);
    ASSERT_EQ(Status, WorkQueue::Status::Success);
    EXPECT_TRUE(Handle.IsAlive());

    EXPECT_EQ(Queue.Run(), WorkQueue::Status::Success);
    EXPECT_EQ(gCountA.load(std::memory_order_acquire), 0);

    std::this_thread::sleep_for(std::chrono::milliseconds(45));
    EXPECT_EQ(Queue.Run(), WorkQueue::Status::Success);
    EXPECT_EQ(gCountA.load(std::memory_order_acquire), 1);
    EXPECT_FALSE(Handle.IsAlive());
}

TEST(MockWorkQueueApi, CancelBeforeFirstRunPreventsExecution)
{
    ResetState();
    WorkQueue Queue;
    ASSERT_EQ(Queue.Init(), WorkQueue::Status::Success);

    WorkQueue::SubmitOptions Options;
    Options.Exec.Fn = &CountTaskA;
    Options.Sched.DelayMs = 0;
    Options.Sched.Iterations = WorkQueue::Scheduling::IterationInfinite;

    auto [Status, Handle] = Queue.Submit(Options);
    ASSERT_EQ(Status, WorkQueue::Status::Success);
    EXPECT_TRUE(Handle.IsAlive());

    EXPECT_EQ(Queue.Cancel(Handle), WorkQueue::Status::Success);
    EXPECT_EQ(Queue.Run(), WorkQueue::Status::Success);
    EXPECT_EQ(gCountA.load(std::memory_order_acquire), 0);
    EXPECT_FALSE(Handle.IsAlive());
}

TEST(MockWorkQueueApi, PriorityOrderingExecutesHigherPriorityFirst)
{
    ResetState();
    WorkQueue Queue;
    ASSERT_EQ(Queue.Init(), WorkQueue::Status::Success);

    int LowId = 1;
    int HighId = 2;

    WorkQueue::SubmitOptions Options;
    Options.Exec.Fn = &OrderTask;
    Options.Sched.DelayMs = 0;
    Options.Sched.Iterations = 1;

    Options.Exec.Ctx = &LowId;
    Options.Exec.PriorityValue = WorkQueue::Priority::Low;
    ASSERT_EQ(Queue.Submit(Options).first, WorkQueue::Status::Success);

    Options.Exec.Ctx = &HighId;
    Options.Exec.PriorityValue = WorkQueue::Priority::High;
    ASSERT_EQ(Queue.Submit(Options).first, WorkQueue::Status::Success);

    EXPECT_EQ(Queue.Run(), WorkQueue::Status::Success);

    std::scoped_lock Lock(gOrderMutex);
    ASSERT_EQ(gOrder.size(), static_cast<size_t>(2));
    EXPECT_EQ(gOrder[0], 2);
    EXPECT_EQ(gOrder[1], 1);
}

TEST(MockWorkQueueApi, EqualPriorityPreservesSubmitOrder)
{
    ResetState();
    WorkQueue Queue;
    ASSERT_EQ(Queue.Init(), WorkQueue::Status::Success);

    int FirstId = 10;
    int SecondId = 20;

    WorkQueue::SubmitOptions Options;
    Options.Exec.Fn = &OrderTask;
    Options.Exec.PriorityValue = WorkQueue::Priority::Normal;
    Options.Sched.DelayMs = 0;
    Options.Sched.Iterations = 1;

    Options.Exec.Ctx = &FirstId;
    ASSERT_EQ(Queue.Submit(Options).first, WorkQueue::Status::Success);

    Options.Exec.Ctx = &SecondId;
    ASSERT_EQ(Queue.Submit(Options).first, WorkQueue::Status::Success);

    EXPECT_EQ(Queue.Run(), WorkQueue::Status::Success);

    std::scoped_lock Lock(gOrderMutex);
    ASSERT_EQ(gOrder.size(), static_cast<size_t>(2));
    EXPECT_EQ(gOrder[0], 10);
    EXPECT_EQ(gOrder[1], 20);
}

TEST(MockWorkQueueApi, DeinitClearsTasksAndInvalidatesHandle)
{
    WorkQueue Queue;
    ASSERT_EQ(Queue.Init(), WorkQueue::Status::Success);

    WorkQueue::SubmitOptions Options;
    Options.Exec.Fn = &CountTaskA;
    Options.Sched.DelayMs = 0;
    Options.Sched.Iterations = WorkQueue::Scheduling::IterationInfinite;

    auto [Status, Handle] = Queue.Submit(Options);
    ASSERT_EQ(Status, WorkQueue::Status::Success);
    EXPECT_TRUE(Handle.IsAlive());

    EXPECT_EQ(Queue.Deinit(), WorkQueue::Status::Success);
    ExpectInvalidHandle(Handle);
}

TEST(MockWorkQueueApi, InitAndDeinitAreRepeatable)
{
    WorkQueue Queue;
    EXPECT_EQ(Queue.Init(), WorkQueue::Status::Success);
    EXPECT_EQ(Queue.Init(), WorkQueue::Status::Success);
    EXPECT_EQ(Queue.Deinit(), WorkQueue::Status::Success);
    EXPECT_EQ(Queue.Deinit(), WorkQueue::Status::Success);
}

#endif // !(WORK_QUEUE_PREEMPTIVE)

// ============================================================================
// PREEMPTIVE MODE TESTS
// ============================================================================

#if WORK_QUEUE_PREEMPTIVE

TEST(PreemptiveWorkQueueApi, SubmitRejectsNullFunction)
{
    WorkQueue Queue;
    ASSERT_EQ(Queue.Init(), WorkQueue::Status::Success);

    WorkQueue::SubmitOptions Options;
    Options.Exec.Fn = nullptr;
    Options.Sched.Iterations = 1;

    auto [Status, Handle] = Queue.Submit(Options);
    EXPECT_EQ(Status, WorkQueue::Status::InvalidParam);
    ExpectInvalidHandle(Handle);
    EXPECT_EQ(Queue.Deinit(), WorkQueue::Status::Success);
}

TEST(PreemptiveWorkQueueApi, SubmitRejectsZeroIterations)
{
    WorkQueue Queue;
    ASSERT_EQ(Queue.Init(), WorkQueue::Status::Success);

    WorkQueue::SubmitOptions Options;
    Options.Exec.Fn = &CountTaskA;
    Options.Sched.Iterations = 0;

    auto [Status, Handle] = Queue.Submit(Options);
    EXPECT_EQ(Status, WorkQueue::Status::InvalidParam);
    ExpectInvalidHandle(Handle);
    EXPECT_EQ(Queue.Deinit(), WorkQueue::Status::Success);
}

TEST(PreemptiveWorkQueueApi, CancelOnInvalidHandleSucceeds)
{
    WorkQueue Queue;
    ASSERT_EQ(Queue.Init(), WorkQueue::Status::Success);

    WorkQueue::WorkHandle InvalidHandle;
    ExpectInvalidHandle(InvalidHandle);
    EXPECT_EQ(Queue.Cancel(InvalidHandle), WorkQueue::Status::Success);
    EXPECT_EQ(Queue.Deinit(), WorkQueue::Status::Success);
}

TEST(PreemptiveWorkQueueApi, UserCanCreateInvalidHandle)
{
    WorkQueue::WorkHandle InvalidHandle;

    ExpectInvalidHandle(InvalidHandle);

    WorkQueue Queue;
    ASSERT_EQ(Queue.Init(), WorkQueue::Status::Success);
    EXPECT_EQ(Queue.Cancel(InvalidHandle), WorkQueue::Status::Success);
    EXPECT_EQ(Queue.Deinit(), WorkQueue::Status::Success);
}

TEST(PreemptiveWorkQueueApi, BasicSubmitAndCompletion)
{
    ResetState();
    WorkQueue Queue;
    ASSERT_EQ(Queue.Init(), WorkQueue::Status::Success);

    WorkQueue::SubmitOptions Options;
    Options.Exec.Fn = &CountTaskA;
    Options.Sched.DelayMs = 0;
    Options.Sched.Iterations = 3;

    auto [Status, Handle] = Queue.Submit(Options);
    ASSERT_EQ(Status, WorkQueue::Status::Success);

    EXPECT_TRUE(WaitUntil(
        []() { return gCountA.load(std::memory_order_acquire) == 3; },
        std::chrono::milliseconds(500)));
    EXPECT_FALSE(Handle.IsAlive());
    EXPECT_EQ(Queue.Deinit(), WorkQueue::Status::Success);
}

TEST(PreemptiveWorkQueueApi, DelayPreventsEarlyExecution)
{
    ResetState();
    WorkQueue Queue;
    ASSERT_EQ(Queue.Init(), WorkQueue::Status::Success);

    WorkQueue::SubmitOptions Options;
    Options.Exec.Fn = &CountTaskA;
    Options.Sched.DelayMs = 80;
    Options.Sched.Iterations = 1;

    auto [Status, Handle] = Queue.Submit(Options);
    ASSERT_EQ(Status, WorkQueue::Status::Success);

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    EXPECT_EQ(gCountA.load(std::memory_order_acquire), 0);
    EXPECT_TRUE(Handle.IsAlive());

    EXPECT_TRUE(WaitUntil(
        []() { return gCountA.load(std::memory_order_acquire) == 1; },
        std::chrono::milliseconds(500)));
    EXPECT_FALSE(Handle.IsAlive());
    EXPECT_EQ(Queue.Deinit(), WorkQueue::Status::Success);
}

TEST(PreemptiveWorkQueueApi, CancelBeforeDueTimePreventsExecution)
{
    ResetState();
    WorkQueue Queue;
    ASSERT_EQ(Queue.Init(), WorkQueue::Status::Success);

    WorkQueue::SubmitOptions Options;
    Options.Exec.Fn = &CountTaskA;
    Options.Sched.DelayMs = 120;
    Options.Sched.Iterations = WorkQueue::Scheduling::IterationInfinite;

    auto [Status, Handle] = Queue.Submit(Options);
    ASSERT_EQ(Status, WorkQueue::Status::Success);

    EXPECT_EQ(Queue.Cancel(Handle), WorkQueue::Status::Success);

    std::this_thread::sleep_for(std::chrono::milliseconds(170));
    EXPECT_EQ(gCountA.load(std::memory_order_acquire), 0);
    EXPECT_FALSE(Handle.IsAlive());
    EXPECT_EQ(Queue.Deinit(), WorkQueue::Status::Success);
}

TEST(PreemptiveWorkQueueApi, ContextIsVisibleFromHandleAndCallback)
{
    WorkQueue Queue;
    ASSERT_EQ(Queue.Init(), WorkQueue::Status::Success);

    int CtxCounter = 0;

    WorkQueue::SubmitOptions Options;
    Options.Exec.Fn = &ContextCounterTask;
    Options.Exec.Ctx = &CtxCounter;
    Options.Sched.DelayMs = 0;
    Options.Sched.Iterations = 2;

    auto [Status, Handle] = Queue.Submit(Options);
    ASSERT_EQ(Status, WorkQueue::Status::Success);
    EXPECT_EQ(Handle.GetContext(), &CtxCounter);

    EXPECT_TRUE(WaitUntil(
        [&CtxCounter]() { return CtxCounter == 2; },
        std::chrono::milliseconds(500)));
    EXPECT_FALSE(Handle.IsAlive());
    EXPECT_EQ(Queue.Deinit(), WorkQueue::Status::Success);
}

TEST(PreemptiveWorkQueueApi, PriorityOrderingExecutesHigherPriorityFirst)
{
    ResetState();
    WorkQueue Queue;
    ASSERT_EQ(Queue.Init(), WorkQueue::Status::Success);

    int LowId = 1;
    int HighId = 2;

    WorkQueue::SubmitOptions Options;
    Options.Exec.Fn = &OrderTask;
    Options.Sched.DelayMs = 0;
    Options.Sched.Iterations = 1;

    Options.Exec.Ctx = &LowId;
    Options.Exec.PriorityValue = WorkQueue::Priority::Low;
    ASSERT_EQ(Queue.Submit(Options).first, WorkQueue::Status::Success);

    Options.Exec.Ctx = &HighId;
    Options.Exec.PriorityValue = WorkQueue::Priority::High;
    ASSERT_EQ(Queue.Submit(Options).first, WorkQueue::Status::Success);

    EXPECT_TRUE(WaitUntil(
        []()
        {
            std::scoped_lock Lock(gOrderMutex);
            return gOrder.size() == 2;
        },
        std::chrono::milliseconds(500)));

    {
        std::scoped_lock Lock(gOrderMutex);
        ASSERT_EQ(gOrder.size(), static_cast<size_t>(2));
        EXPECT_EQ(gOrder[0], 2);
        EXPECT_EQ(gOrder[1], 1);
    }

    EXPECT_EQ(Queue.Deinit(), WorkQueue::Status::Success);
}

TEST(PreemptiveWorkQueueApi, DeinitClearsTasksAndInvalidatesHandle)
{
    WorkQueue Queue;
    ASSERT_EQ(Queue.Init(), WorkQueue::Status::Success);

    WorkQueue::SubmitOptions Options;
    Options.Exec.Fn = &CountTaskA;
    Options.Sched.DelayMs = 0;
    Options.Sched.Iterations = WorkQueue::Scheduling::IterationInfinite;

    auto [Status, Handle] = Queue.Submit(Options);
    ASSERT_EQ(Status, WorkQueue::Status::Success);
    ASSERT_TRUE(Handle.IsAlive());

    EXPECT_EQ(Queue.Deinit(), WorkQueue::Status::Success);
    ExpectInvalidHandle(Handle);
}

TEST(PreemptiveWorkQueueApi, InitAndDeinitAreRepeatable)
{
    WorkQueue Queue;
    EXPECT_EQ(Queue.Init(), WorkQueue::Status::Success);
    EXPECT_EQ(Queue.Init(), WorkQueue::Status::Success);
    EXPECT_EQ(Queue.Deinit(), WorkQueue::Status::Success);
    EXPECT_EQ(Queue.Deinit(), WorkQueue::Status::Success);
}

#endif // WORK_QUEUE_PREEMPTIVE