// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "AP_Rust_config.h"

#if AP_SCRIPTING_RUST_ENABLED
#include <stddef.h>
#include <stdint.h>

namespace AP_Rust
{

class Image
{
public:
    struct Compatibility {
        uint32_t machine;
        uint32_t abi_version;
        uint32_t cpu_profile;
        uint32_t stack_limit;
        uint8_t fingerprint[16];
    };

    // Source bytes must stay alive and immutable through load(). No allocation
    // or executable-memory permission changes are performed by this class.
    // A failed load may leave partial output; never publish or execute it.
    bool parse(const uint8_t *source, size_t length, uint32_t memory_limit, const Compatibility &compatibility);
#if AP_RUST_LEGACY_PROBE_ENABLED
    // Legacy relocation probes have no API identity. Only test builds expose
    // this path; runtime module discovery must use the strict parse().
    bool parse_probe(const uint8_t *source, size_t length, uint32_t memory_limit, uint32_t machine);
#endif
    using Resolver = uint32_t (*)(void *context, uint64_t hash);
    bool check_imports(Resolver resolver, void *context = nullptr, uint64_t *missing = nullptr) const;
    // UINT32_MAX rejects an unknown contract. Resolve only before publication.
    bool load(void *destination, size_t size, Resolver resolver = nullptr, void *context = nullptr) const;

    uint32_t memory_size() const
    {
        return _memory_size;
    }
    uint32_t code_size() const
    {
        return _code_size;
    }
    uint32_t entry_offset() const
    {
        return _entry;
    }
    // Declared module stack use, not a measured bound. The caller must budget
    // backend overhead (including POSIX TLS/libc) separately. Zero for probes.
    uint32_t required_stack_size() const
    {
        return _stack_size;
    }

private:
    static uint32_t word(const uint8_t *pointer);
    bool parse_internal(const uint8_t *source, size_t length, uint32_t memory_limit,
                        uint32_t machine, const Compatibility *compatibility);
    const uint8_t *_source{};
    size_t _length{};
    uint32_t _file_size{};
    uint32_t _memory_size{};
    uint32_t _code_size{};
    uint32_t _entry{};
    uint32_t _count{};
    uint32_t _width{};
    uint32_t _header_size{};
    uint32_t _stack_size{};
    uint32_t _import_count{};
};

}
#endif // AP_SCRIPTING_RUST_ENABLED
