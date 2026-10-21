// SPDX-License-Identifier: GPL-3.0-or-later
#include "runtime.hpp"
#include "new"

namespace {
const AP_ModuleHostApi *host;
void *host_context;
void *allocate(size_t n, size_t alignment, bool may_fail)
{
    void *p = nullptr;
    if (n <= UINT32_MAX && alignment <= 4096) {
        p = host->allocate(host_context, uint32_t(n == 0 ? 1 : n), uint32_t(alignment));
    }
    if (p == nullptr && !may_fail) { ap_module::fatal("C++ allocation failed"); }
    return p;
}
void deallocate(void *p) { if (p != nullptr) { host->deallocate(host_context, static_cast<uint8_t *>(p)); } }
}
extern "C" int32_t ap_module_run(const ap_module::ContextBase *context);
extern "C" int32_t start(const AP_ModuleHostApi *api, void *raw)
{
    if (api == nullptr || api->version != AP_MODULE_ABI_VERSION || api->size < sizeof(*api)) {
        return AP_MODULE_STATUS_INVALID;
    }
    const uint8_t expected[] = AP_MODULE_API_FINGERPRINT;
    for (size_t i = 0; i < sizeof(expected); i++) {
        if (api->fingerprint[i] != expected[i]) { return AP_MODULE_STATUS_INVALID; }
    }
    if (api->allocate == nullptr || api->deallocate == nullptr || api->time_us == nullptr ||
        api->sleep_ms == nullptr || api->stop_requested == nullptr || api->text == nullptr ||
        api->panic == nullptr || api->binding_call == nullptr) { return AP_MODULE_STATUS_INVALID; }
    host = api;
    host_context = raw;
    const ap_module::ContextBase context(*api, raw);
    return ap_module_run(&context);
}
[[noreturn]] void ap_module::fatal(const char *message)
{
    uint32_t length = 0;
    while (length < 512 && message[length] != 0) { length++; }
    host->panic(host_context, reinterpret_cast<const uint8_t *>(message), length);
    __builtin_trap();
}
void *operator new(size_t n) { return allocate(n, alignof(max_align_t), false); }
void *operator new[](size_t n) { return ::operator new(n); }
void *operator new(size_t n, const std::nothrow_t &) noexcept { return allocate(n, alignof(max_align_t), true); }
void *operator new[](size_t n, const std::nothrow_t &t) noexcept { return ::operator new(n, t); }
void *operator new(size_t n, std::align_val_t a) { return allocate(n, size_t(a), false); }
void *operator new[](size_t n, std::align_val_t a) { return ::operator new(n, a); }
void *operator new(size_t n, std::align_val_t a, const std::nothrow_t &) noexcept { return allocate(n, size_t(a), true); }
void *operator new[](size_t n, std::align_val_t a, const std::nothrow_t &t) noexcept { return ::operator new(n, a, t); }
void operator delete(void *p) noexcept { deallocate(p); }
void operator delete[](void *p) noexcept { deallocate(p); }
void operator delete(void *p, size_t) noexcept { deallocate(p); }
void operator delete[](void *p, size_t) noexcept { deallocate(p); }
void operator delete(void *p, std::align_val_t) noexcept { deallocate(p); }
void operator delete[](void *p, std::align_val_t) noexcept { deallocate(p); }
void operator delete(void *p, size_t, std::align_val_t) noexcept { deallocate(p); }
void operator delete[](void *p, size_t, std::align_val_t) noexcept { deallocate(p); }
extern "C" void *memcpy(void *dst, const void *src, size_t n)
{
    auto *d = static_cast<uint8_t *>(dst);
    const auto *s = static_cast<const uint8_t *>(src);
    for (size_t i = 0; i < n; i++) { d[i] = s[i]; }
    return dst;
}
extern "C" void *memmove(void *dst, const void *src, size_t n)
{
    auto *d = static_cast<uint8_t *>(dst);
    const auto *s = static_cast<const uint8_t *>(src);
    if (reinterpret_cast<uintptr_t>(d) <= reinterpret_cast<uintptr_t>(s)) { return memcpy(dst, src, n); }
    for (size_t i = n; i != 0; i--) { d[i - 1] = s[i - 1]; }
    return dst;
}
extern "C" void *memset(void *dst, int c, size_t n)
{
    auto *d = static_cast<uint8_t *>(dst);
    for (size_t i = 0; i < n; i++) { d[i] = uint8_t(c); }
    return dst;
}
extern "C" int memcmp(const void *a, const void *b, size_t n)
{
    const auto *x = static_cast<const uint8_t *>(a);
    const auto *y = static_cast<const uint8_t *>(b);
    for (size_t i = 0; i < n; i++) { if (x[i] != y[i]) { return int(x[i]) - int(y[i]); } }
    return 0;
}
extern "C" void __cxa_pure_virtual() { ap_module::fatal("C++ pure virtual call"); }
#if defined(__arm__)
extern "C" void __aeabi_memcpy(void *d, const void *s, size_t n) { memcpy(d, s, n); }
extern "C" void __aeabi_memcpy4(void *d, const void *s, size_t n) { memcpy(d, s, n); }
extern "C" void __aeabi_memcpy8(void *d, const void *s, size_t n) { memcpy(d, s, n); }
extern "C" void __aeabi_memclr(void *d, size_t n) { memset(d, 0, n); }
extern "C" void __aeabi_memclr4(void *d, size_t n) { memset(d, 0, n); }
extern "C" void __aeabi_memclr8(void *d, size_t n) { memset(d, 0, n); }
#endif
