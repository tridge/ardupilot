// SPDX-License-Identifier: GPL-3.0-or-later
#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#if HAL_OWNED_THREADS_ENABLED && (CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX)
#include <AP_HAL/utility/OwnedThread_posix.h>
#include <atomic>
#include <unistd.h>

class OwnedThreadTask
{
public:
    ~OwnedThreadTask()
    {
        release.store(true);
        if (thread != nullptr) {
            if (!reap()) {
                // Do not destroy callback context while a stuck worker uses it.
                abort();
            }
        }
    }

    bool start(uint32_t size = 65536)
    {
        thread = hal.scheduler->thread_create_owned(FUNCTOR_BIND_MEMBER(&OwnedThreadTask::run, void),
                 "owned-test", size, AP_HAL::Scheduler::PRIORITY_SCRIPTING, 0);
        return thread != nullptr;
    }

    bool reap()
    {
        for (uint32_t i = 0; i < 10000; i++) {
            if (AP_HAL::Scheduler::thread_reap(thread)) {
                return true;
            }
            usleep(1000);
        }
        return false;
    }

    void run()
    {
        pthread_attr_t attr;
        void *stack;
        size_t size;
        if (pthread_getattr_np(pthread_self(), &attr) == 0) {
            if (pthread_attr_getstack(&attr, &stack, &size) == 0) {
                actual_stack_size = size;
            }
            pthread_attr_destroy(&attr);
        }
        entered.store(true);
        while (!release.load()) {
            usleep(1000);
        }
        result = 42;
    }

    AP_HAL::Scheduler::OwnedThread *thread = nullptr;
    std::atomic<bool> release{false};
    std::atomic<bool> entered{false};
    uint32_t actual_stack_size = 0;
    uint32_t result = 0;
};

TEST(OwnedThread, RunningThreadsRetainResources)
{
    OwnedThreadTask first, second;
    ASSERT_TRUE(first.start());
    ASSERT_TRUE(second.start());
    auto *first_handle = first.thread;
    auto *second_handle = second.thread;
    EXPECT_FALSE(AP_HAL::Scheduler::thread_reap(first.thread));
    EXPECT_FALSE(AP_HAL::Scheduler::thread_reap(second.thread));
    EXPECT_EQ(first.thread, first_handle);
    EXPECT_EQ(second.thread, second_handle);
    const uint32_t stack_size = first.thread->stack_size();
    EXPECT_GE(stack_size, 65536U);
    EXPECT_GT(first.thread->allocation_size(), stack_size);
    first.release.store(true);
    ASSERT_TRUE(first.reap());
    EXPECT_EQ(first.thread, nullptr);
    EXPECT_EQ(first.result, 42U);
    EXPECT_EQ(first.actual_stack_size, stack_size);
    EXPECT_FALSE(AP_HAL::Scheduler::thread_reap(first.thread));
    EXPECT_FALSE(AP_HAL::Scheduler::thread_reap(second.thread));
    second.release.store(true);
    ASSERT_TRUE(second.reap());
    EXPECT_EQ(second.result, 42U);
}

TEST(OwnedThread, RepeatedImmediateExitAndMinimumStack)
{
    for (uint32_t i = 0; i < 100; i++) {
        OwnedThreadTask task;
        task.release.store(true);
        ASSERT_TRUE(task.start(1024));
        const uint32_t reserved = task.thread->stack_size();
        EXPECT_GE(reserved, uint32_t(PTHREAD_STACK_MIN));
        ASSERT_TRUE(task.reap());
        EXPECT_EQ(task.result, 42U);
        EXPECT_EQ(task.actual_stack_size, reserved);
    }
}

TEST(OwnedThread, RejectInvalidRequests)
{
    OwnedThreadTask task;
    EXPECT_FALSE(task.start(0));
    EXPECT_FALSE(task.start(UINT32_MAX));
    EXPECT_EQ(hal.scheduler->thread_create_owned(AP_HAL::MemberProc{}, "empty", 65536,
              AP_HAL::Scheduler::PRIORITY_SCRIPTING, 0), nullptr);
    // Exercise cleanup after allocation but before successful pthread_create.
    const auto proc = FUNCTOR_BIND(&task, &OwnedThreadTask::run, void);
    for (uint32_t i = 0; i < 100; i++) {
        EXPECT_EQ(OwnedThreadPosix::create(proc, "invalid-policy", 65536, SCHED_OTHER, 99), nullptr);
    }
    ASSERT_TRUE(task.start());
    task.release.store(true);
    EXPECT_TRUE(task.reap());
}
#else
TEST(OwnedThread, RequiresEnabledPosixBuild)
{
    GTEST_SKIP() << "Requires HAL_OWNED_THREADS_ENABLED on Linux/SITL";
}
#endif

AP_GTEST_MAIN()
