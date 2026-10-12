// SPDX-License-Identifier: GPL-3.0-or-later
#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Rust/Heap.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#if AP_SCRIPTING_RUST_ENABLED
TEST(RustHeap, AlignmentBoundsAndCleanup)
{
    AP_Rust::Heap heap;
    ASSERT_TRUE(heap.create(32768));
    EXPECT_FALSE(heap.create(32768));
    EXPECT_EQ(heap.allocate(0, 8), nullptr);
    EXPECT_EQ(heap.allocate(1, 0), nullptr);
    EXPECT_EQ(heap.allocate(1, 3), nullptr);
    EXPECT_EQ(heap.allocate(1, 8192), nullptr);
    EXPECT_EQ(heap.allocate(UINT32_MAX, 8), nullptr);
    for (uint32_t alignment = 1; alignment <= 4096; alignment *= 2) {
        void *pointer = heap.allocate(37, alignment);
        ASSERT_NE(pointer, nullptr);
        EXPECT_EQ(reinterpret_cast<uintptr_t>(pointer) % alignment, 0U);
        memset(pointer, 0xA5, 37);
    }
    EXPECT_EQ(heap.stats().allocations, 13U);
    EXPECT_GT(heap.stats().used, 13U * 37);
    heap.clear();
    EXPECT_EQ(heap.stats().used, 0U);
    EXPECT_EQ(heap.stats().allocations, 0U);
    EXPECT_GT(heap.stats().peak, 0U);
}

TEST(RustHeap, ExhaustionDoesNotExpand)
{
    AP_Rust::Heap heap;
    ASSERT_TRUE(heap.create(4096));
    void *pointers[64] {};
    uint8_t count = 0;
    while (count < ARRAY_SIZE(pointers)) {
        void *pointer = heap.allocate(100, 16);
        if (pointer == nullptr) {
            break;
        }
        pointers[count++] = pointer;
    }
    EXPECT_GT(count, 0U);
    EXPECT_LT(count, ARRAY_SIZE(pointers));
    EXPECT_LE(heap.stats().used, 4096U);
    // MultiHeap's second failed allocation normally permits expansion.
    EXPECT_EQ(heap.allocate(100, 16), nullptr);
    EXPECT_EQ(heap.allocate(100, 16), nullptr);
    int foreign;
    EXPECT_FALSE(heap.deallocate(&foreign));
    for (uint8_t i = 0; i < count; i++) {
        ASSERT_TRUE(heap.deallocate(pointers[i]));
        EXPECT_FALSE(heap.deallocate(pointers[i]));
    }
    EXPECT_EQ(heap.stats().used, 0U);
    void *pointer = heap.allocate(100, 16);
    ASSERT_NE(pointer, nullptr);
    // Destruction must reclaim an outstanding allocation on the host backend.
}
#else
TEST(RustHeap, RequiresEnabledBuild)
{
    GTEST_SKIP() << "Configure with --extra-hwdef libraries/AP_Rust/tests/enabled.hwdef";
}
#endif // AP_SCRIPTING_RUST_ENABLED

AP_GTEST_MAIN()
