// SPDX-License-Identifier: GPL-3.0-or-later
// Exercise the actual manager using an emulator-owned SD card.
#include <AP_HAL/AP_HAL.h>
#include <AP_Rust/Runtime.h>
#include <AP_Rust/Bindings.h>
#include <AP_Rust/BindingConfig.h>
#include <AP_Rust/HostApi.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_Common/ExpandingString.h>
#include <GCS_MAVLink/GCS_Dummy.h>
#include <atomic>
#include <stdio.h>
#include <stdlib.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

volatile uint32_t rust_runtime_test_result;
volatile uint32_t rust_runtime_test_cycles;
volatile uint32_t rust_runtime_test_before;
volatile uint32_t rust_runtime_test_after;
volatile uint32_t rust_runtime_test_stack_free;
volatile uint32_t rust_runtime_test_stack_total;
volatile uint32_t rust_runtime_test_manager_priority;
volatile uint32_t rust_runtime_test_worker_priority;
volatile uint32_t rust_runtime_test_manager_stack;
volatile uint32_t rust_runtime_test_worker_stack;
volatile uint32_t rust_runtime_test_timeouts;
volatile uint32_t rust_runtime_test_timeout_elapsed;
volatile uint32_t rust_runtime_test_panics;
char rust_runtime_test_error[100];
char rust_runtime_test_memory[256];

#if AP_SCRIPTING_RUST_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
class RuntimeGCS : public GCS_Dummy
{
public:
    std::atomic<uint32_t> loaded_reports{0};
    std::atomic<uint32_t> loaded_count{0};
    std::atomic<uint32_t> timeouts{0};
    std::atomic<uint32_t> panics{0};
    std::atomic<uint32_t> parameter_ready{0};
    std::atomic<uint32_t> parameter_restored{0};

    void send_textv(MAV_SEVERITY severity, const char *format, va_list args,
                    mavlink_channel_mask_t destination) override
    {
        char message[160];
        vsnprintf(message, sizeof(message), format, args);
        if (severity <= MAV_SEVERITY_ERROR) {
            hal.util->snprintf(rust_runtime_test_error, sizeof(rust_runtime_test_error), "%s", message);
            ExpandingString memory(rust_runtime_test_memory, sizeof(rust_runtime_test_memory));
            hal.util->mem_info(memory);
        }
        if (strncmp(message, "Rust: loaded ", 13) == 0 && message[13] >= '0' && message[13] <= '8') {
            loaded_count.store(message[13] - '0');
            loaded_reports.fetch_add(1);
        }
        if (strstr(message, "stop timeout") != nullptr) {
            timeouts.fetch_add(1);
        }
        if (strstr(message, "panic; reboot required") != nullptr) {
            panics.fetch_add(1);
        }
        if (strstr(message, "PARAM ready") != nullptr) {
            parameter_ready.fetch_add(1);
        }
        if (strstr(message, "PARAM restored") != nullptr) {
            parameter_restored.fetch_add(1);
        }
    }
};

static RuntimeGCS test_gcs;
static AP_Rust::Runtime runtime;
static const AP_Param::Info parameter_info[] = { AP_VAREND };
static AP_Param parameter_loader{parameter_info};

class RuntimeHarness
{
public:
    void run();

private:
    bool set(const char *name, float value)
    {
        return AP_Param::set_object_value(&runtime, AP_Rust::Runtime::var_info, name, value);
    }
    bool wait_loaded(uint32_t previous, uint32_t count)
    {
        const uint32_t start = AP_HAL::millis();
        while (AP_HAL::millis() - start < 5000) {
            if (test_gcs.loaded_reports.load() != previous) {
                return test_gcs.loaded_count.load() == count;
            }
            hal.scheduler->delay(1);
        }
        return false;
    }
    bool restart(uint32_t count)
    {
        const uint32_t previous = test_gcs.loaded_reports.load();
        const uint32_t ready = test_gcs.parameter_ready.load();
        if (!set("RESTART", ++_request)) {
            return false;
        }
        runtime.update();
        if (!wait_loaded(previous, count)) {
            return false;
        }
        if (count == 2) {
            const uint32_t start = AP_HAL::millis();
            while (test_gcs.parameter_ready.load() == ready) {
                if (AP_HAL::millis() - start >= 5000) {
                    return false;
                }
                hal.scheduler->delay(1);
            }
        }
        return true;
    }
    bool exercise();
    uint16_t _request{};
};

static RuntimeHarness harness;

static bool thread_values(const char *line, uint32_t &priority, uint32_t &free_stack,
                          uint32_t &total_stack, uint32_t &address)
{
    if (line == nullptr) {
        return false;
    }
    const char *end = strchr(line, '\n');
    const char *pri = strstr(line, "PRI=");
    const char *stack = strstr(line, "STACK=");
    const char *pointer = strstr(line, "sp=");
    if (end == nullptr || pri == nullptr || stack == nullptr || pointer == nullptr ||
        pri >= end || stack >= end || pointer >= end) {
        return false;
    }
    priority = strtoul(pri + 4, nullptr, 10);
    address = strtoul(pointer + 3, nullptr, 16);
    char *separator;
    free_stack = strtoul(stack + 6, &separator, 10);
    if (*separator != '/' || separator >= end) {
        return false;
    }
    total_stack = strtoul(separator + 1, nullptr, 10);
    return true;
}

bool RuntimeHarness::exercise()
{
    // A wire pointer with a nonzero high word must not truncate to a valid
    // STM32 address. No C++ object is constructed on a rejected receiver.
    uint8_t input[25] {};
    input[4] = 1;
    input[8] = 'A';
    uint8_t output = 0xFF;
    if (AP_Rust::binding_dispatch(nullptr, AP_RUST_BINDING_PARAMETER_INIT,
                                  input, sizeof(input), &output, sizeof(output)) != AP_RUST_STATUS_INVALID || output != 0) {
        return false;
    }
    for (unsigned retry = 0; !AP::FS().retry_mount(); retry++) {
        if (retry == 5) {
            return false;
        }
        hal.scheduler->delay(100);
    }
    rust_runtime_test_result = 2;
    if (!set("ENABLE", 1) || !set("HEAP_SIZE", 4096) || !set("CODE_SIZE", 65536) || !set("STACK_SIZE", 16384)) {
        return false;
    }
    runtime.init();
    char message[100];
    if (!wait_loaded(0, 0) || !runtime.arming_checks(sizeof(message), message)) {
        return false;
    }
    rust_runtime_test_result = 3;
    auto &fs = AP::FS();
    if ((fs.mkdir("/APM") != 0 && errno != EEXIST) || fs.mkdir("/APM/rust") != 0) {
        return false;
    }
    rust_runtime_test_result = 30;
    if (fs.rename("/fixtures/one.apmod", "/APM/rust/one.apmod") != 0 ||
        fs.rename("/fixtures/two.apmod", "/APM/rust/two.apmod") != 0) {
        return false;
    }
    rust_runtime_test_result = 31;
    if (!restart(2)) {
        return false;
    }
    // Warm the release/reallocation path as well as first-time loading before
    // requiring an exact baseline for subsequent generations.
    if (!restart(2)) {
        return false;
    }
    hal.scheduler->delay(100);
    rust_runtime_test_before = hal.util->available_memory();
    for (unsigned cycle = 0; cycle < 20; cycle++) {
        rust_runtime_test_result = 4;
        if (!restart(2)) {
            return false;
        }
        hal.scheduler->delay(100);
        rust_runtime_test_cycles = rust_runtime_test_cycles + 1;
    }
    rust_runtime_test_after = hal.util->available_memory();
    if (rust_runtime_test_before != rust_runtime_test_after || !runtime.arming_checks(sizeof(message), message) ||
        test_gcs.parameter_restored.load() != rust_runtime_test_cycles + 1) {
        return false;
    }
    rust_runtime_test_result = 5;
    if (fs.unlink("/APM/rust/one.apmod") != 0 || fs.unlink("/APM/rust/two.apmod") != 0 ||
        fs.rename("/fixtures/end.apmod", "/APM/rust/one.apmod") != 0 || !restart(1)) {
        return false;
    }
    hal.scheduler->delay(100);
    const uint32_t loaded = test_gcs.loaded_reports.load();
    const uint32_t retained = hal.util->available_memory();
    if (!set("RESTART", ++_request)) {
        return false;
    }
    runtime.update();
    const uint32_t start = AP_HAL::millis();
    while (test_gcs.timeouts.load() == 0 && AP_HAL::millis() - start < 12000) {
        hal.scheduler->delay(10);
    }
    rust_runtime_test_timeout_elapsed = AP_HAL::millis() - start;
    rust_runtime_test_timeouts = test_gcs.timeouts.load();
    rust_runtime_test_panics = test_gcs.panics.load();
    rust_runtime_test_result = 6;
    if (rust_runtime_test_timeouts != 1 || test_gcs.loaded_reports.load() != loaded ||
        runtime.arming_checks(sizeof(message), message) || hal.util->available_memory() != retained) {
        return false;
    }
    if (rust_runtime_test_panics != 0 && strstr(message, "one.apmod panic") == nullptr) {
        return false; // preserve the original module failure across stop timeout
    }
    // Read the HAL's real thread registry; no private Runtime test access.
    ExpandingString threads;
    hal.util->thread_info(threads);
    const char *all_threads = threads.get_string();
    const char *manager = all_threads == nullptr ? nullptr : strstr(all_threads, "RustMgr");
    const char *worker = all_threads == nullptr ? nullptr : strstr(all_threads, "one.apmod");
    uint32_t manager_priority, worker_priority, free_stack, total_stack, worker_free, worker_total;
    uint32_t manager_address, worker_address;
    rust_runtime_test_result = 7;
    if (!thread_values(manager, manager_priority, free_stack, total_stack, manager_address) ||
        !thread_values(worker, worker_priority, worker_free, worker_total, worker_address)) {
        return false;
    }
    rust_runtime_test_stack_free = free_stack;
    rust_runtime_test_stack_total = total_stack;
    rust_runtime_test_manager_priority = manager_priority;
    rust_runtime_test_worker_priority = worker_priority;
    rust_runtime_test_manager_stack = manager_address;
    rust_runtime_test_worker_stack = worker_address;
    return manager_priority > worker_priority && free_stack > 4096;
}

void RuntimeHarness::run()
{
    if (exercise()) {
        rust_runtime_test_result = 0x600DF00D;
    }
    while (true) {
        hal.scheduler->delay(1000);
    }
}
#endif

void setup();
void loop();

void setup()
{
    rust_runtime_test_result = 1;
#if AP_SCRIPTING_RUST_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    AP_Param::setup();
    AP_Param::load_all();
    // Test driver only: it sleeps while waiting for the lower-priority manager.
    if (!hal.scheduler->thread_create(FUNCTOR_BIND(&harness, &RuntimeHarness::run, void), "RustTest", 8192,
                                      AP_HAL::Scheduler::PRIORITY_SCRIPTING, 2)) {
        rust_runtime_test_result = 0xBAD0;
    }
#else
    rust_runtime_test_result = 0xBAD1;
#endif
}

void loop()
{
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
