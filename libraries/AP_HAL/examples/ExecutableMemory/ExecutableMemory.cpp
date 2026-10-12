// SPDX-License-Identifier: GPL-3.0-or-later
// Execute tiny native functions from two independent HAL-owned RAM images.
#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

volatile uint32_t executable_memory_test_result;
volatile uint32_t executable_memory_test_before;
volatile uint32_t executable_memory_test_after;
volatile uintptr_t executable_memory_test_address;

#if HAL_EXECUTABLE_MEMORY_ENABLED && (defined(__ARM_ARCH_7EM__) || defined(__x86_64__))
static bool exercise(uint8_t answer)
{
#if defined(__ARM_ARCH_7EM__)
    const auto expected_target = AP_HAL::ExecutableMemory::Target::ThumbV7EMHardFloat;
#else
    const auto expected_target = AP_HAL::ExecutableMemory::Target::X86_64Baseline;
#endif
    if (hal.util->executable_memory_target() != expected_target) {
        executable_memory_test_result = 6;
        return false;
    }
    AP_HAL::ExecutableMemory *images[2] {};
    bool passed = true;
    for (uint8_t i = 0; i < 2; i++) {
        images[i] = hal.util->allocate_executable(8192);
        if (images[i] == nullptr) {
            executable_memory_test_result = 2;
            passed = false;
            break;
        }
        auto *data = static_cast<uint8_t *>(images[i]->data());
        executable_memory_test_address = reinterpret_cast<uintptr_t>(data);
#if defined(__ARM_ARCH_7EM__)
        // Thumb: movs r0, #answer; bx lr.
        const uint8_t code[] = {uint8_t(answer + i), 0x20, 0x70, 0x47};
#else
        // x86-64: mov eax, answer; ret.
        const uint8_t code[] = {0xB8, uint8_t(answer + i), 0, 0, 0, 0xC3};
#endif
        memcpy(data, code, sizeof(code));
        if (!images[i]->publish(4096)) {
            executable_memory_test_result = 3;
            passed = false;
            break;
        }
        data[4096] = answer + i;
    }
    if (passed) {
        for (uint8_t i = 0; i < 2; i++) {
            uintptr_t entry = reinterpret_cast<uintptr_t>(images[i]->data());
#if defined(__ARM_ARCH_7EM__)
            entry |= 1; // Thumb function pointer.
#endif
            using Entry = uint32_t (*)();
            const uint32_t value = reinterpret_cast<Entry>(entry)();
            if (value != uint32_t(answer + i) || static_cast<uint8_t *>(images[i]->data())[4096] != value) {
                executable_memory_test_result = 4;
                passed = false;
            }
        }
    }
    // Both functions have returned; neither image has any executing users.
    delete images[0];
    delete images[1];
    return passed;
}
#endif

void setup();
void loop();

void setup()
{
    executable_memory_test_result = 1;
#if HAL_EXECUTABLE_MEMORY_ENABLED && (defined(__ARM_ARCH_7EM__) || defined(__x86_64__))
    if (!exercise(42)) {
        return;
    }
    executable_memory_test_before = hal.util->available_memory();
    for (uint8_t i = 0; i < 100; i++) {
        if (!exercise(i + 42)) {
            return;
        }
    }
    executable_memory_test_after = hal.util->available_memory();
    if (executable_memory_test_before > executable_memory_test_after &&
        executable_memory_test_before - executable_memory_test_after > 1024) {
        executable_memory_test_result = 5;
        return;
    }
    executable_memory_test_result = 0x600DF00D;
#else
    executable_memory_test_result = 0xBAD0;
#endif
}

void loop()
{
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
