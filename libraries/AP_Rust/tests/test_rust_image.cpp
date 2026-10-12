// SPDX-License-Identifier: GPL-3.0-or-later
#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/crc.h>
#include <AP_Rust/Image.h>
#include <AP_Rust/Heap.h>

#if AP_SCRIPTING_RUST_ENABLED && defined(__linux__) && defined(__x86_64__)
#include <memory>
#include <vector>
#endif

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#if AP_SCRIPTING_RUST_ENABLED
namespace
{

struct Container {
    uint8_t bytes[56 + 4104 + 8] {};

    void put(uint32_t offset, uint32_t value)
    {
        for (uint8_t i = 0; i < 4; i++) {
            bytes[offset + i] = uint8_t(value >> (8 * i));
        }
    }

    void checksum()
    {
        put(52, 0);
        put(52, crc32_small(UINT32_MAX, bytes, sizeof(bytes)) ^ UINT32_MAX);
    }

    Container()
    {
        memcpy(bytes, "APRUST01", 8);
        put(8, 1);
        put(12, sizeof(uintptr_t) == 8 ? 62 : 40);
        put(16, sizeof(uintptr_t));
        put(20, 4096);
        put(24, 4104);
        put(28, 4120);
        put(32, 4096);
        put(36, sizeof(uintptr_t) == 8 ? 0 : 1);
        put(40, 1);
        bytes[56] = 0xA5;
        put(56 + 4104, 4096);
        put(56 + 4104 + 4, 8);
        checksum();
    }

#if AP_RUST_LEGACY_PROBE_ENABLED
    bool parse(AP_Rust::Image &image, uint32_t limit = 8192)
    {
        return image.parse_probe(bytes, sizeof(bytes), limit, sizeof(uintptr_t) == 8 ? 62 : 40);
    }
#endif
};

struct RuntimeContainer {
    uint8_t bytes[80 + 4104 + 8] {};

    static AP_Rust::Image::Compatibility compatibility()
    {
        return {sizeof(uintptr_t) == 8 ? 62U : 40U, 1,
                sizeof(uintptr_t) == 8 ? 2U : 1U, 8192,
        {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16}};
    }

    void put(uint32_t offset, uint32_t value)
    {
        for (uint8_t i = 0; i < 4; i++) {
            bytes[offset + i] = uint8_t(value >> (8 * i));
        }
    }

    void checksum()
    {
        put(76, 0);
        put(76, crc32_small(UINT32_MAX, bytes, sizeof(bytes)) ^ UINT32_MAX);
    }

    RuntimeContainer()
    {
        Container probe;
        memcpy(bytes, probe.bytes, 44);
        memcpy(bytes, "APRUST03", 8);
        put(8, 3);
        const auto expected = compatibility();
        put(44, expected.abi_version);
        put(48, expected.cpu_profile);
        put(52, 4096);
        memcpy(bytes + 60, expected.fingerprint, sizeof(expected.fingerprint));
        memcpy(bytes + 80, probe.bytes + 56, sizeof(probe.bytes) - 56);
        checksum();
    }
};

struct ImportContainer {
    uint8_t bytes[80 + 4112 + 8 + 24] {};

    void put(uint32_t offset, uint32_t value)
    {
        for (uint8_t i = 0; i < 4; i++) {
            bytes[offset + i] = uint8_t(value >> (8 * i));
        }
    }

    void checksum()
    {
        put(76, 0);
        put(76, crc32_small(UINT32_MAX, bytes, sizeof(bytes)) ^ UINT32_MAX);
    }

    ImportContainer()
    {
        RuntimeContainer base;
        memcpy(bytes, base.bytes, 80 + 4104);
        put(24, 4112);
        put(28, 4128);
        put(56, 2);
        put(80 + 4104, UINT32_MAX);
        put(80 + 4108, UINT32_MAX);
        put(80 + 4112, 4096);
        put(80 + 4112 + 4, 8);
        const uint32_t records = 80 + 4112 + 8;
        put(records, 0x55667788);
        put(records + 4, 0x11223344);
        put(records + 8, 4104);
        put(records + 12, 0x44332211);
        put(records + 16, 0x88776655);
        put(records + 20, 4108);
        checksum();
    }

    static uint32_t resolve(void *context, uint64_t hash)
    {
        const uint32_t offset = *static_cast<uint32_t *>(context);
        if (hash == UINT64_C(0x1122334455667788)) {
            return offset + 77;
        }
        if (hash == UINT64_C(0x8877665544332211) && offset != UINT16_MAX) {
            return offset + 88;
        }
        return UINT32_MAX;
    }
};

}

TEST(RustImage, ResolveImportsBeforePublication)
{
    ImportContainer container;
    AP_Rust::Image image;
    ASSERT_TRUE(image.parse(container.bytes, sizeof(container.bytes), 8192, RuntimeContainer::compatibility()));
    alignas(4096) uint8_t destination[8192];
    memset(destination, 0x5A, sizeof(destination));
    EXPECT_FALSE(image.load(destination, sizeof(destination)));
    uint32_t offset = UINT16_MAX; // first known, second unknown
    EXPECT_FALSE(image.load(destination, sizeof(destination), ImportContainer::resolve, &offset));
    for (const auto byte : destination) {
        ASSERT_EQ(byte, 0x5A);
    }
    // Exactly the same source image works with changed firmware-private IDs.
    for (const uint32_t firmware_offset : {
             0U, 1000U
         }) {
        offset = firmware_offset;
        ASSERT_TRUE(image.load(destination, sizeof(destination), ImportContainer::resolve, &offset));
        uint32_t first, second;
        memcpy(&first, destination + 4104, 4);
        memcpy(&second, destination + 4108, 4);
        EXPECT_EQ(first, offset + 77);
        EXPECT_EQ(second, offset + 88);
        EXPECT_EQ(destination[4112], 0);
    }
}

TEST(RustImage, RejectImportMetadataErrors)
{
    const uint32_t records = 80 + 4112 + 8;
    const struct {
        uint32_t offset;
        uint32_t value;
    } mutations[] = {
        {56, 0}, {56, UINT32_MAX}, {56, 4097},
        {records + 8, 4092}, {records + 8, 4096}, {records + 8, 4105},
        {records + 20, 4104}, {records + 20, 4112}, {records + 20, UINT32_MAX},
        {80 + 4104, 0}, {80 + 4108, 0},
    };
    AP_Rust::Image image;
    for (const auto &mutation : mutations) {
        ImportContainer container;
        container.put(mutation.offset, mutation.value);
        container.checksum();
        EXPECT_FALSE(image.parse(container.bytes, sizeof(container.bytes), 8192, RuntimeContainer::compatibility()));
    }
    ImportContainer duplicate;
    memcpy(duplicate.bytes + records + 12, duplicate.bytes + records, 8);
    duplicate.checksum();
    EXPECT_FALSE(image.parse(duplicate.bytes, sizeof(duplicate.bytes), 8192, RuntimeContainer::compatibility()));
    ImportContainer corrupt;
    corrupt.bytes[records] ^= 1;
    EXPECT_FALSE(image.parse(corrupt.bytes, sizeof(corrupt.bytes), 8192, RuntimeContainer::compatibility()));
    ImportContainer old;
    memcpy(old.bytes, "APRUST02", 8);
    old.put(8, 2);
    old.checksum();
    EXPECT_FALSE(image.parse(old.bytes, sizeof(old.bytes), 8192, RuntimeContainer::compatibility()));
}

TEST(RustImage, RequireRuntimeCompatibility)
{
    AP_Rust::Image image;
    auto expected = RuntimeContainer::compatibility();
    Container probe;
    EXPECT_FALSE(image.parse(probe.bytes, sizeof(probe.bytes), 8192, expected));
    RuntimeContainer container;
#if AP_RUST_LEGACY_PROBE_ENABLED
    EXPECT_FALSE(image.parse_probe(container.bytes, sizeof(container.bytes), 8192, expected.machine));
#endif
    ASSERT_TRUE(image.parse(container.bytes, sizeof(container.bytes), 8192, expected));
    EXPECT_EQ(image.required_stack_size(), 4096U);
    alignas(4096) uint8_t destination[8192];
    ASSERT_TRUE(image.load(destination, sizeof(destination)));
    EXPECT_EQ(destination[0], 0xA5);
    uintptr_t pointer;
    memcpy(&pointer, destination + 4096, sizeof(pointer));
    EXPECT_EQ(pointer, reinterpret_cast<uintptr_t>(destination + 8));
    EXPECT_EQ(destination[4104], 0);

    expected.stack_limit = 4095;
    EXPECT_FALSE(image.parse(container.bytes, sizeof(container.bytes), 8192, expected));
    EXPECT_FALSE(image.load(destination, sizeof(destination)));
    expected.stack_limit = 4096;
    EXPECT_TRUE(image.parse(container.bytes, sizeof(container.bytes), 8192, expected));
    expected.fingerprint[15] ^= 1;
    EXPECT_FALSE(image.parse(container.bytes, sizeof(container.bytes), 8192, expected));
}

TEST(RustImage, AcceptByteGranularStackRequirements)
{
    auto expected = RuntimeContainer::compatibility();
    for (const uint32_t size : {
             256U, 257U, 4095U, 4097U, 16U * 1024 * 1024
         }) {
        RuntimeContainer container;
        container.put(52, size);
        container.checksum();
        AP_Rust::Image image;
        expected.stack_limit = size;
        ASSERT_TRUE(image.parse(container.bytes, sizeof(container.bytes), 8192, expected));
        EXPECT_EQ(image.required_stack_size(), size);
        expected.stack_limit = size - 1;
        EXPECT_FALSE(image.parse(container.bytes, sizeof(container.bytes), 8192, expected));
    }
}

TEST(RustImage, RejectRuntimeMetadataErrors)
{
    const auto expected = RuntimeContainer::compatibility();
    const struct {
        uint32_t offset;
        uint32_t value;
    } mutations[] = {
        {8, 1}, {44, 0}, {44, 2}, {48, 0}, {48, 3U - expected.cpu_profile},
        {52, 0}, {52, 255}, {52, 8193}, {52, UINT32_MAX}, {56, 1},
        {60, 0}, {80 + 4104, 4103}, {80 + 4104 + 4, 4121}, {56, UINT32_MAX},
    };
    AP_Rust::Image image;
    for (const auto &mutation : mutations) {
        RuntimeContainer container;
        container.put(mutation.offset, mutation.value);
        container.checksum();
        EXPECT_FALSE(image.parse(container.bytes, sizeof(container.bytes), 8192, expected)) << mutation.offset;
    }
    RuntimeContainer container;
    for (const size_t length : {
             size_t(0), size_t(55), size_t(79), sizeof(container.bytes) - 1
         }) {
        EXPECT_FALSE(image.parse(container.bytes, length, 8192, expected));
    }
    container.bytes[80] ^= 1;
    EXPECT_FALSE(image.parse(container.bytes, sizeof(container.bytes), 8192, expected));
    EXPECT_FALSE(image.parse(nullptr, sizeof(container.bytes), 8192, expected));
}

#if AP_RUST_LEGACY_PROBE_ENABLED
TEST(RustImage, RelocateAndZeroBss)
{
    Container container;
    AP_Rust::Image image;
    ASSERT_TRUE(container.parse(image));
    alignas(4096) uint8_t destination[8192];
    memset(destination, 0xFF, sizeof(destination));
    EXPECT_FALSE(image.load(destination + 1, sizeof(destination) - 1));
    EXPECT_FALSE(image.load(destination, 4000));
    ASSERT_TRUE(image.load(destination, sizeof(destination)));
    EXPECT_EQ(destination[0], 0xA5);
    uintptr_t pointer;
    memcpy(&pointer, destination + 4096, sizeof(pointer));
    EXPECT_EQ(pointer, reinterpret_cast<uintptr_t>(destination + 8));
    for (uint16_t i = 4104; i < 4120; i++) {
        EXPECT_EQ(destination[i], 0);
    }
    EXPECT_EQ(destination[4120], 0xFF);
}

TEST(RustImage, RejectMalformed)
{
    AP_Rust::Image image;
    Container container;
    EXPECT_FALSE(container.parse(image, 4119));
    container.bytes[56] ^= 1;
    EXPECT_FALSE(container.parse(image));
    container.checksum();
    EXPECT_TRUE(container.parse(image));
    container.put(56 + 4104, 4103);
    container.checksum();
    EXPECT_FALSE(container.parse(image));
    alignas(4096) uint8_t destination[8192];
    EXPECT_FALSE(image.load(destination, sizeof(destination)));
    container.put(56 + 4104, 4096);
    container.put(56 + 4104 + 4, 4121);
    container.checksum();
    EXPECT_FALSE(container.parse(image));
}

TEST(RustImage, RejectBoundsAndProfileErrors)
{
    const struct {
        uint32_t offset;
        uint32_t value;
    } mutations[] = {
        {0, 0},
        {12, 0},
        {16, sizeof(uintptr_t) == 8 ? 4U : 8U},
        {24, UINT32_MAX},
        {36, UINT32_MAX},
        {56 + 4104, 4104},
        {56 + 4096, 1},
    };
    AP_Rust::Image image;
    for (const auto &mutation : mutations) {
        Container container;
        container.put(mutation.offset, mutation.value);
        container.checksum();
        EXPECT_FALSE(container.parse(image)) << mutation.offset;
    }
    alignas(4096) Container container;
    EXPECT_FALSE(image.parse_probe(container.bytes, sizeof(container.bytes) - 1, 8192, sizeof(uintptr_t) == 8 ? 62 : 40));
    ASSERT_TRUE(container.parse(image));
    EXPECT_FALSE(image.load(container.bytes, sizeof(container.bytes)));
    EXPECT_FALSE(image.load(reinterpret_cast<void *>(UINTPTR_MAX & ~uintptr_t(4095)), 8192));
    container.put(56 + 4104, UINT32_MAX);
    alignas(4096) uint8_t destination[8192];
    EXPECT_FALSE(image.load(destination, sizeof(destination)));
}

#endif // AP_RUST_LEGACY_PROBE_ENABLED

#if HAL_EXECUTABLE_MEMORY_ENABLED && defined(__linux__) && defined(__x86_64__)
namespace
{
AP_Rust::Heap *probe_heap;
uint32_t outputs;
uint32_t sleeps;
bool callback_error;

void probe_output(const uint8_t *text, size_t size)
{
    callback_error |= size != 17 || memcmp(text, "Rust module probe", 17) != 0;
    outputs++;
}

void probe_sleep(uint32_t ms)
{
    sleeps += ms;
}

void *probe_allocate(size_t size, size_t alignment)
{
    if (size > UINT32_MAX || alignment > UINT32_MAX) {
        return nullptr;
    }
    return probe_heap->allocate(size, alignment);
}

void probe_deallocate(void *pointer, size_t size, size_t alignment)
{
    callback_error |= !probe_heap->deallocate(pointer);
}
}

static void run_toolchain_probe(bool runtime)
{
    const char *path = getenv(runtime ? "AP_RUST_TEST_RUNTIME_MODULE" : "AP_RUST_TEST_MODULE");
    if (path == nullptr) {
        GTEST_SKIP() << "Set AP_RUST_TEST_MODULE and AP_RUST_TEST_RUNTIME_MODULE to the packed x86-64 probes";
    }
    FILE *file = fopen(path, "rb");
    ASSERT_NE(file, nullptr);
    ASSERT_EQ(fseek(file, 0, SEEK_END), 0);
    const long length = ftell(file);
    ASSERT_GT(length, 0);
    ASSERT_LT(length, 1024 * 1024);
    rewind(file);
    std::vector<uint8_t> source(length);
    const size_t received = fread(source.data(), 1, source.size(), file);
    fclose(file);
    ASSERT_EQ(received, source.size());
    AP_Rust::Image image;
    if (runtime) {
        ASSERT_TRUE(image.parse(source.data(), source.size(), 1024 * 1024, RuntimeContainer::compatibility()));
    } else {
#if AP_RUST_LEGACY_PROBE_ENABLED
        ASSERT_TRUE(image.parse_probe(source.data(), source.size(), 1024 * 1024, 62));
#else
        GTEST_SKIP() << "Requires AP_RUST_LEGACY_PROBE_ENABLED";
#endif
    }
    AP_Rust::Heap heap;
    ASSERT_TRUE(heap.create(32768));
    probe_heap = &heap;
    outputs = sleeps = 0;
    callback_error = false;
    const struct {
        decltype(&probe_output) output;
        decltype(&probe_sleep) sleep;
        decltype(&probe_allocate) allocate;
        decltype(&probe_deallocate) deallocate;
    } host{probe_output, probe_sleep, probe_allocate, probe_deallocate};
    std::unique_ptr<AP_HAL::ExecutableMemory> memory(hal.util->allocate_executable(image.memory_size()));
    ASSERT_NE(memory, nullptr);
    const bool loaded = image.load(memory->data(), memory->size());
    const bool published = loaded && memory->publish(image.code_size());
    EXPECT_TRUE(published);
    if (published) {
        using Start = uint32_t (*)(const void *);
        const auto start = reinterpret_cast<Start>(static_cast<uint8_t *>(memory->data()) + image.entry_offset());
        EXPECT_EQ(start(&host), 31U);
        EXPECT_EQ(start(&host), 26U);
        EXPECT_EQ(outputs, 2U);
        EXPECT_EQ(sleeps, 20U);
        EXPECT_FALSE(callback_error);
        EXPECT_EQ(heap.stats().used, 0U);
        EXPECT_GT(heap.stats().peak, 256U);
    }
    probe_heap = nullptr;
}

TEST(RustImage, ToolchainProbeWithMultiHeap)
{
    run_toolchain_probe(false);
}

TEST(RustImage, ToolchainRuntimeContainerWithMultiHeap)
{
    run_toolchain_probe(true);
}
#elif defined(__linux__) && defined(__x86_64__)
TEST(RustImage, ToolchainProbeWithMultiHeap)
{
    GTEST_SKIP() << "Requires HAL_EXECUTABLE_MEMORY_ENABLED";
}
#endif
#else
TEST(RustImage, RequiresEnabledBuild)
{
    GTEST_SKIP() << "Configure with --extra-hwdef libraries/AP_Rust/tests/enabled.hwdef";
}
#endif // AP_SCRIPTING_RUST_ENABLED

AP_GTEST_MAIN()
