// SPDX-License-Identifier: GPL-3.0-or-later
#include <ap_module.hpp>
#include <new>

#ifndef MODULE_PROBE_TEXT
#define MODULE_PROBE_TEXT "ready"
#endif

namespace {
uint32_t generation = 41;
struct Counter {
    virtual uint32_t next() = 0;
    virtual ~Counter() = default;
};
struct OwnedCounter : Counter {
    uint8_t *memory = new uint8_t[512];
    uint32_t next() override { return generation++; }
    ~OwnedCounter() override { delete[] memory; }
};
__attribute__((noinline)) uint32_t next(Counter &c) { return c.next(); }
}

static ap_module::Result<void> run(ap_module::Context &context)
{
    OwnedCounter counter;
    if (next(counter) != 41) { return {ap_module::Error::Host}; }
    ap_module::Vector3f vector{3.0f, 4.0f, 0.0f};
    auto length = context.vector3f().length(vector);
    if (!length || length.value != 5.0f) { return {ap_module::Error::Host}; }
    auto sent = context.text(6, MODULE_PROBE_TEXT);
    if (!sent) { return sent; }
    while (!context.stop_requested()) {
        const auto slept = context.sleep_ms(1000);
        if (!slept && slept.error != ap_module::Error::Stopped) { return slept; }
    }
    return {ap_module::Error::Ok};
}
AP_MODULE(run)
