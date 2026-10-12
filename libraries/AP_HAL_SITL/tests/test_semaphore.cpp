// SPDX-License-Identifier: GPL-3.0-or-later
#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <AP_HAL_SITL/Semaphores.h>
#include <new>

class InspectSemaphore : public HALSITL::Semaphore
{
public:
    uint8_t recursion() const
    {
        return take_count;
    }
};

TEST(SITLSemaphore, InitializesRecursionWithoutZeroedStorage)
{
    alignas(InspectSemaphore) uint8_t storage[sizeof(InspectSemaphore)];
    memset(storage, 0xA5, sizeof(storage));
    auto *semaphore = new (storage) InspectSemaphore;
    EXPECT_EQ(semaphore->recursion(), 0U);
    ASSERT_TRUE(semaphore->take_nonblocking());
    EXPECT_EQ(semaphore->recursion(), 1U);
    ASSERT_TRUE(semaphore->take_nonblocking());
    EXPECT_EQ(semaphore->recursion(), 2U);
    EXPECT_TRUE(semaphore->give());
    EXPECT_EQ(semaphore->recursion(), 1U);
    semaphore->check_owner();
    EXPECT_TRUE(semaphore->give());
    EXPECT_EQ(semaphore->recursion(), 0U);
    semaphore->~InspectSemaphore();
}
#else
TEST(SITLSemaphore, RequiresSITL)
{
    GTEST_SKIP() << "Requires SITL";
}
#endif

AP_GTEST_MAIN()
