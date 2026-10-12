// SPDX-License-Identifier: GPL-3.0-or-later
#include "Image.h"

#if AP_SCRIPTING_RUST_ENABLED
#include <AP_Math/crc.h>
#include <string.h>

namespace AP_Rust
{

static constexpr uint32_t PROBE_HEADER_SIZE = 56;
static constexpr uint32_t RUNTIME_HEADER_SIZE = 80;
static constexpr uint32_t ALIGNMENT = 4096;
static constexpr uint32_t MAX_IMAGE = 16U * 1024 * 1024;
static constexpr uint32_t MAX_IMPORTS = 4096;

uint32_t Image::word(const uint8_t *pointer)
{
    return uint32_t(pointer[0]) | uint32_t(pointer[1]) << 8 |
           uint32_t(pointer[2]) << 16 | uint32_t(pointer[3]) << 24;
}

bool Image::parse(const uint8_t *source, size_t length, uint32_t memory_limit, const Compatibility &compatibility)
{
    return parse_internal(source, length, memory_limit, compatibility.machine, &compatibility);
}

#if AP_RUST_LEGACY_PROBE_ENABLED
bool Image::parse_probe(const uint8_t *source, size_t length, uint32_t memory_limit, uint32_t machine)
{
    return parse_internal(source, length, memory_limit, machine, nullptr);
}
#endif

bool Image::parse_internal(const uint8_t *source, size_t length, uint32_t memory_limit,
                           uint32_t machine, const Compatibility *compatibility)
{
    *this = Image{};
    const uint32_t header_size = compatibility == nullptr ? PROBE_HEADER_SIZE : RUNTIME_HEADER_SIZE;
    const char *magic = compatibility == nullptr ? "APRUST01" : "APRUST03";
    const uint32_t version = compatibility == nullptr ? 1 : 3;
    if (source == nullptr || length < header_size || memcmp(source, magic, 8) != 0 ||
        word(source + 8) != version || word(source + 12) != machine) {
        return false;
    }
    uint32_t stack_size = 0;
    uint32_t import_count = 0;
    if (compatibility != nullptr) {
        const uint32_t profile = word(source + 48);
        stack_size = word(source + 52);
        import_count = word(source + 56);
        const uint8_t empty_fingerprint[16] {};
        if (word(source + 44) == 0 || word(source + 44) != compatibility->abi_version ||
            !((machine == 40 && profile == 1) || (machine == 62 && profile == 2)) ||
            profile != compatibility->cpu_profile ||
            stack_size < 256 || stack_size > MAX_IMAGE ||
            stack_size > compatibility->stack_limit ||
            memcmp(source + 60, empty_fingerprint, sizeof(empty_fingerprint)) == 0 ||
            memcmp(source + 60, compatibility->fingerprint, sizeof(compatibility->fingerprint)) != 0) {
            return false;
        }
    } else if (word(source + 44) != 0 || word(source + 48) != 0) {
        return false;
    }
    const uint32_t width = word(source + 16);
    const uint32_t file_size = word(source + 24);
    const uint32_t memory_size = word(source + 28);
    const uint32_t code_size = word(source + 32);
    const uint32_t entry = word(source + 36);
    const uint32_t count = word(source + 40);
    if (!((machine == 40 && width == 4) || (machine == 62 && width == 8)) ||
        width != sizeof(uintptr_t) || word(source + 20) != ALIGNMENT ||
        file_size == 0 ||
        file_size > memory_size || memory_size > memory_limit || memory_size > MAX_IMAGE ||
        code_size == 0 || code_size % ALIGNMENT != 0 ||
        code_size > ((memory_size + ALIGNMENT - 1) & ~(ALIGNMENT - 1)) ||
        entry >= code_size || entry >= file_size || (machine == 40 && (entry & 1) == 0) ||
        count > file_size / width || import_count > file_size / 4 || import_count > MAX_IMPORTS ||
        length != header_size + size_t(file_size) + size_t(count) * 8 + size_t(import_count) * 12) {
        return false;
    }
    const uint8_t zeros[4] {};
    uint32_t crc = crc32_small(UINT32_MAX, source, header_size - 4);
    crc = crc32_small(crc, zeros, sizeof(zeros));
    crc = crc32_small(crc, source + header_size, length - header_size);
    if ((crc ^ UINT32_MAX) != word(source + header_size - 4)) {
        return false;
    }
    uint32_t previous_end = 0;
    for (uint32_t i = 0; i < count; i++) {
        const uint8_t *fixup = source + header_size + file_size + i * 8;
        const uint32_t offset = word(fixup);
        if (offset < previous_end || offset % width != 0 || file_size < width ||
            offset > file_size - width || word(fixup + 4) > memory_size) {
            return false;
        }
        for (uint32_t j = 0; j < width; j++) {
            if (source[header_size + offset + j] != 0) {
                return false;
            }
        }
        previous_end = offset + width;
    }
    previous_end = code_size;
    uint32_t fixup_index = 0;
    const uint8_t *imports = source + header_size + file_size + count * 8;
    for (uint32_t i = 0; i < import_count; i++) {
        const uint32_t offset = word(imports + i * 12 + 8);
        if (offset < previous_end || offset % 4 != 0 || file_size < 4 || offset > file_size - 4 ||
            word(source + header_size + offset) != UINT32_MAX) {
            return false;
        }
        for (uint32_t j = 0; j < i; j++) {
            if (memcmp(imports + j * 12, imports + i * 12, 8) == 0) {
                return false;
            }
        }
        // Both lists are offset-sorted. Slots must not overlap pointer fixups.
        while (fixup_index < count && word(source + header_size + file_size + fixup_index * 8) + width <= offset) {
            fixup_index++;
        }
        if (fixup_index < count && word(source + header_size + file_size + fixup_index * 8) < offset + 4) {
            return false;
        }
        previous_end = offset + 4;
    }
    _source = source;
    _length = length;
    _file_size = file_size;
    _memory_size = memory_size;
    _code_size = code_size;
    _entry = entry;
    _count = count;
    _width = width;
    _header_size = header_size;
    _stack_size = stack_size;
    _import_count = import_count;
    return true;
}

bool Image::check_imports(Resolver resolver, void *context, uint64_t *missing) const
{
    if (_source == nullptr) {
        return false;
    }
    for (uint32_t i = 0; i < _import_count; i++) {
        const uint8_t *entry = _source + _header_size + _file_size + _count * 8 + i * 12;
        const uint64_t hash = uint64_t(word(entry)) | (uint64_t(word(entry + 4)) << 32);
        if (resolver == nullptr || resolver(context, hash) == UINT32_MAX) {
            if (missing != nullptr) {
                *missing = hash;
            }
            return false;
        }
    }
    return true;
}

bool Image::load(void *destination, size_t size, Resolver resolver, void *context) const
{
    const uintptr_t address = reinterpret_cast<uintptr_t>(destination);
    const uintptr_t source_address = reinterpret_cast<uintptr_t>(_source);
    if (_source == nullptr || destination == nullptr || size < _memory_size || address % ALIGNMENT != 0 ||
        address > UINTPTR_MAX - _memory_size || source_address > UINTPTR_MAX - _length ||
        (address < source_address + _length && source_address < address + _memory_size) ||
        (_import_count != 0 && resolver == nullptr)) {
        return false;
    }
    auto *bytes = static_cast<uint8_t *>(destination);
    // A missing import must reject the image before any output is modified.
    // Resolvers are immutable firmware metadata; the second lookup below must
    // return the same slot. No allocation or per-module dispatch table needed.
    if (!check_imports(resolver, context)) {
        return false;
    }
    memcpy(bytes, _source + _header_size, _file_size);
    memset(bytes + _file_size, 0, _memory_size - _file_size);
    for (uint32_t i = 0; i < _count; i++) {
        const uint8_t *fixup = _source + _header_size + _file_size + i * 8;
        const uint32_t offset = word(fixup);
        const uint32_t target = word(fixup + 4);
        if (_file_size < _width || offset > _file_size - _width || offset % _width != 0 || target > _memory_size) {
            return false;
        }
        const uintptr_t value = address + target;
        for (uint32_t j = 0; j < _width; j++) {
            bytes[offset + j] = uint8_t(value >> (j * 8));
        }
    }
    for (uint32_t i = 0; i < _import_count; i++) {
        const uint8_t *entry = _source + _header_size + _file_size + _count * 8 + i * 12;
        const uint64_t hash = uint64_t(word(entry)) | (uint64_t(word(entry + 4)) << 32);
        const uint32_t slot = resolver(context, hash);
        if (slot == UINT32_MAX) {
            return false;
        }
        const uint32_t offset = word(entry + 8);
        for (uint32_t j = 0; j < 4; j++) {
            bytes[offset + j] = uint8_t(slot >> (j * 8));
        }
    }
    return true;
}

}
#endif // AP_SCRIPTING_RUST_ENABLED
