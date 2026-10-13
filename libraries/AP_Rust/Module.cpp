// SPDX-License-Identifier: GPL-3.0-or-later
#include "Module.h"

#if AP_SCRIPTING_RUST_ENABLED && HAL_OWNED_THREADS_ENABLED && HAL_EXECUTABLE_MEMORY_ENABLED
#include "Image.h"
#include <AP_Math/crc.h>
#include <string.h>

extern const AP_HAL::HAL &hal;

namespace AP_Rust
{

const AP_RustHostApi Module::_api {
    AP_RUST_ABI_VERSION, sizeof(AP_RustHostApi), AP_RUST_API_FINGERPRINT,
    allocate, deallocate, time_us, sleep_ms, stop_requested, text, panic, binding_call
};

uint32_t Module::rounded_code_size(uint32_t requested)
{
    if (requested < 4096 || requested > 16U * 1024 * 1024) {
        return 0;
    }
    return (requested + 4095U) & ~4095U;
}

Module *Module::load(const uint8_t *source, size_t length, const char *name,
                     const Limits &limits, TextSink sink, Error &error, BindingSink bindings, uint64_t *missing_import)
{
    constexpr uint32_t maximum = 16U * 1024 * 1024;
    const uint32_t code_size = rounded_code_size(limits.code);
    error = Error::Limits;
    if (name == nullptr || name[0] == '\0' || limits.heap < 256 || limits.heap > maximum ||
        code_size == 0 ||
        limits.stack < 4096 || limits.stack > maximum) {
        return nullptr;
    }
    Image::Compatibility expected{0, AP_RUST_ABI_VERSION, 0, 0, AP_RUST_API_FINGERPRINT};
    uint32_t margin = 0;
    error = Error::Target;
    switch (hal.util->executable_memory_target()) {
    case AP_HAL::ExecutableMemory::Target::ThumbV7EMHardFloat:
        expected.machine = 40;
        expected.cpu_profile = 1;
        // Provisional host callback margin, including the TextSink contract.
        // Measure the complete binding set before enabling vehicle modules.
        margin = 4096;
        break;
    case AP_HAL::ExecutableMemory::Target::X86_64Baseline:
        expected.machine = 62;
        expected.cpu_profile = 2;
        // Includes libc/TLS use and host callbacks, not a stack-usage proof.
        margin = 16384;
        break;
    default:
        return nullptr;
    }
    error = Error::Limits;
    if (limits.stack <= margin) {
        return nullptr;
    }
    expected.stack_limit = limits.stack - margin;
    Image image;
    error = Error::Image;
    if (!image.parse(source, length, code_size, expected)) {
        return nullptr;
    }
    error = Error::Imports;
    if (!image.check_imports(bindings.resolve, bindings.context, missing_import)) {
        return nullptr;
    }
    error = Error::Memory;
    auto *module = NEW_NOTHROW Module(limits, sink, bindings);
    if (module == nullptr) {
        return nullptr;
    }
    strncpy(module->_name, name, sizeof(module->_name) - 1);
    // Reserve the more restricted executable-capable RAM first. Ordinary
    // heap storage may use memory (for example F405 CCM) that code cannot.
    module->_memory = hal.util->allocate_executable(image.memory_size());
    if (module->_memory == nullptr) {
        delete module;
        return nullptr;
    }
    error = Error::Publication;
    if (!image.load(module->_memory->data(), module->_memory->size(), bindings.resolve, bindings.context) ||
        !module->_memory->publish(image.code_size())) {
        delete module;
        return nullptr;
    }
    module->_entry = reinterpret_cast<Start>(static_cast<uint8_t *>(module->_memory->data()) + image.entry_offset());
    module->_checksum = crc32_small(UINT32_MAX, source, length) ^ UINT32_MAX;
    error = Error::None;
    return module;
}

Module::~Module()
{
    delete _memory;
}

bool Module::start()
{
    if (_started || _stop.load()) {
        return false;
    }
    // Prepare every generation's images before starting its workers. Deferring
    // ordinary heaps until here keeps them out of scarce executable SRAM while
    // those images are being reserved. A failed thread start can reuse its heap.
    if (!_heap_ready) {
        if (!_heap.create(_limits.heap, _limits.allow_heap_expansion)) {
            return false;
        }
        _heap_ready = true;
    }
    _thread = hal.scheduler->thread_create_owned(FUNCTOR_BIND_MEMBER(&Module::run, void), _name,
              _limits.stack, AP_HAL::Scheduler::PRIORITY_SCRIPTING, 0);
    _started = _thread != nullptr;
    return _started;
}

void Module::run()
{
    _result = _entry(&_api, this);
    _returned.store(true, std::memory_order_release);
}

void Module::request_stop()
{
    _stop.store(true);
}

bool Module::poll_join()
{
    return _thread == nullptr || AP_HAL::Scheduler::thread_reap(_thread);
}

bool Module::release(Module *&module)
{
    if (module == nullptr) {
        return true;
    }
    if (!module->poll_join()) {
        return false;
    }
    delete module;
    module = nullptr;
    return true;
}

Module::State Module::state() const
{
    if (_panicked.load(std::memory_order_acquire)) {
        return State::Panicked;
    }
    if (!_started) {
        return State::Prepared;
    }
    if (_thread == nullptr) {
        return State::Joined;
    }
    return _returned.load(std::memory_order_acquire) ? State::Returned : State::Running;
}

int32_t Module::result() const
{
    return _returned.load(std::memory_order_acquire) ? _result : AP_RUST_STATUS_UNAVAILABLE;
}

const char *Module::panic_message() const
{
    return _panicked.load(std::memory_order_acquire) ? _panic_message : nullptr;
}

Module::Stats Module::stats()
{
    return {_heap.stats(), _limits.heap, _memory->allocation_size(),
            _thread == nullptr ? 0 : _thread->stack_size(),
            _thread == nullptr ? 0 : _thread->allocation_size(), sizeof(*this)};
}

uint8_t *Module::allocate(void *context, uint32_t size, uint32_t alignment)
{
    auto &module = *static_cast<Module *>(context);
    // Rust destructors and in-flight work can still allocate after stop. A
    // synthetic OOM here would turn cooperative exit into a permanent panic.
    return static_cast<uint8_t *>(module._heap.allocate(size, alignment));
}

void Module::deallocate(void *context, uint8_t *allocation)
{
    auto &module = *static_cast<Module *>(context);
    if (!module._heap.deallocate(allocation)) {
        static const uint8_t message[] = "invalid deallocation";
        panic(context, message, sizeof(message) - 1);
    }
}

uint64_t Module::time_us(void *context)
{
    return AP_HAL::micros64();
}

int32_t Module::sleep_ms(void *context, uint32_t milliseconds)
{
    auto &module = *static_cast<Module *>(context);
    const uint64_t start = AP_HAL::millis64();
    do {
        if (module._stop.load()) {
            return AP_RUST_STATUS_STOPPED;
        }
        const uint64_t elapsed = AP_HAL::millis64() - start;
        if (elapsed >= milliseconds) {
            return AP_RUST_STATUS_OK;
        }
        const uint32_t remaining = milliseconds - uint32_t(elapsed);
        hal.scheduler->delay(remaining > 10 ? 10 : remaining);
    } while (true);
}

uint8_t Module::stop_requested(void *context)
{
    return static_cast<Module *>(context)->_stop.load();
}

int32_t Module::text(void *context, uint8_t severity, const uint8_t *message, uint32_t length)
{
    auto &module = *static_cast<Module *>(context);
    if (module._stop.load()) {
        return AP_RUST_STATUS_STOPPED;
    }
    if (message == nullptr || severity > 7 || length > 512) {
        return AP_RUST_STATUS_INVALID;
    }
    if (module._sink.send == nullptr) {
        return AP_RUST_STATUS_UNAVAILABLE;
    }
    module._sink.send(module._sink.context, severity, reinterpret_cast<const char *>(message), length);
    return AP_RUST_STATUS_OK;
}

int32_t Module::binding_call(void *context, uint32_t id, const uint8_t *input, uint32_t input_size,
                             uint8_t *output, uint32_t output_size)
{
    auto &module = *static_cast<Module *>(context);
    if (module._stop.load()) {
        return AP_RUST_STATUS_STOPPED;
    }
    if (module._bindings.call == nullptr) {
        return AP_RUST_STATUS_UNAVAILABLE;
    }
    return module._bindings.call(module._bindings.context, id, input, input_size, output, output_size);
}

void Module::panic(void *context, const uint8_t *message, uint32_t length)
{
    auto &module = *static_cast<Module *>(context);
    const uint32_t count = length < sizeof(module._panic_message) - 1 ? length : sizeof(module._panic_message) - 1;
    if (message != nullptr) {
        memcpy(module._panic_message, message, count);
    }
    module._panicked.store(true, std::memory_order_release);
    // This callback never returns. In particular it must not use sleep_ms(),
    // which returns immediately after a stop request. Keep Rust frames alive
    // at the existing scripting priority; only the delayed wakeup runs.
    while (true) {
        hal.scheduler->delay(1000);
    }
}

}
#endif
