// SPDX-License-Identifier: GPL-3.0-or-later
#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Module/Module.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#if AP_MODULE_ENABLED && HAL_OWNED_THREADS_ENABLED && HAL_EXECUTABLE_MEMORY_ENABLED
TEST(RustModuleLimits, RoundCodeBudget)
{
    EXPECT_EQ(AP_Module::Module::rounded_code_size(0), 0U);
    EXPECT_EQ(AP_Module::Module::rounded_code_size(4095), 0U);
    EXPECT_EQ(AP_Module::Module::rounded_code_size(4096), 4096U);
    EXPECT_EQ(AP_Module::Module::rounded_code_size(4097), 8192U);
    EXPECT_EQ(AP_Module::Module::rounded_code_size(131071), 131072U);
    EXPECT_EQ(AP_Module::Module::rounded_code_size(16U * 1024 * 1024 - 1), 16U * 1024 * 1024);
    EXPECT_EQ(AP_Module::Module::rounded_code_size(16U * 1024 * 1024), 16U * 1024 * 1024);
    EXPECT_EQ(AP_Module::Module::rounded_code_size(16U * 1024 * 1024 + 1), 0U);
    EXPECT_EQ(AP_Module::Module::rounded_code_size(UINT32_MAX), 0U);
}
#endif

// Native Linux uses the actual scheduler clock. SITL without a simulated
// vehicle advances its synthetic clock instead of sleeping; use vehicle tests
// there for sleep latency and panic parking rather than a misleading unit test.
#if AP_MODULE_ENABLED && HAL_OWNED_THREADS_ENABLED && HAL_EXECUTABLE_MEMORY_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_LINUX && defined(__x86_64__)
#include <atomic>
#include <string>
#include <vector>
#include <sys/resource.h>
#include <unistd.h>

using AP_Module::Module;

class RustModule : public ::testing::Test
{
protected:
    void SetUp() override
    {
        directory = getenv("AP_MODULE_TEST_MODULE_DIR");
        if (directory == nullptr) {
            GTEST_SKIP() << "Set AP_MODULE_TEST_MODULE_DIR to compiled lifecycle fixtures";
        }
    }
    void TearDown() override
    {
        for (auto &module : modules) {
            if (module != nullptr) {
                module->request_stop();
                if (!join(module) || !Module::release(module)) {
                    abort(); // never let a live worker use dead test context
                }
            }
        }
    }
    static void text(void *context, uint8_t severity, const char *message, uint32_t length)
    {
        auto &test = *static_cast<RustModule *>(context);
        if (severity != 6 || length != 5 || memcmp(message, "ready", 5) != 0) {
            test.bad_text.store(true);
        }
        test.messages.fetch_add(1);
    }
    std::vector<uint8_t> read(const char *name)
    {
        const auto path = std::string(directory) + "/" + name + ".apmod";
        FILE *file = fopen(path.c_str(), "rb");
        if (file == nullptr) {
            ADD_FAILURE() << "missing fixture " << path;
            return {};
        }
        std::vector<uint8_t> bytes;
        uint8_t buffer[4096];
        size_t length;
        while ((length = fread(buffer, 1, sizeof(buffer), file)) != 0 && bytes.size() < 1024 * 1024) {
            bytes.insert(bytes.end(), buffer, buffer + length);
        }
        EXPECT_FALSE(ferror(file));
        fclose(file);
        return bytes;
    }
    Module *load(const char *name, uint32_t heap = 32768,
                 Module::Source source = Module::Source::File)
    {
        const auto bytes = read(name);
        Module::Error error;
        auto *module = Module::load(bytes.data(), bytes.size(), name,
                                    {
                                        heap, 131072, 65536
                                    }, {this, text}, error, {}, nullptr, nullptr, source);
        EXPECT_EQ(error, Module::Error::None);
        return module;
    }
    bool ready(uint32_t count)
    {
        for (unsigned i = 0; i < 10000; i++) {
            if (messages.load() >= count) {
                return !bad_text.load();
            }
            usleep(1000);
        }
        return false;
    }
    static bool join(Module *module)
    {
        for (unsigned i = 0; i < 10000; i++) {
            if (module->poll_join()) {
                return true;
            }
            usleep(1000);
        }
        return false;
    }
    bool wait_panic(Module *module)
    {
        for (unsigned i = 0; i < 10000; i++) {
            if (module->state() == Module::State::Panicked) {
                return true;
            }
            usleep(1000);
        }
        return false;
    }
    void panic_child(const char *name)
    {
        auto *module = load(name, 4096);
        if (module == nullptr || !module->start() || !wait_panic(module)) {
            _exit(1);
        }
        module->request_stop();
        const auto before = module->stats();
        const auto *identity = module;
        struct rusage usage_start, usage_end;
        getrusage(RUSAGE_SELF, &usage_start);
        for (unsigned i = 0; i < 150; i++) {
            if (Module::release(module) || module != identity) {
                _exit(2);
            }
            usleep(10000);
        }
        getrusage(RUSAGE_SELF, &usage_end);
        const auto cpu = [](const struct rusage &usage) {
            return (usage.ru_utime.tv_sec + usage.ru_stime.tv_sec) * int64_t(1000000) +
                   usage.ru_utime.tv_usec + usage.ru_stime.tv_usec;
        };
        const auto after = module->stats();
        if (cpu(usage_end) - cpu(usage_start) > 200000 || before.heap.used == 0 ||
            before.heap.used != after.heap.used || before.rx_reserved != after.rx_reserved ||
            before.rw_reserved != after.rw_reserved ||
            before.thread_reserved != after.thread_reserved || module->panic_message() == nullptr ||
            module->panic_message()[0] == '\0') {
            _exit(3);
        }
        // The child process, not the runtime, ends this intentionally parked
        // thread. No unsafe unload/cancellation escape hatch is provided.
        _exit(73);
    }

    const char *directory{};
    Module *modules[2] {};
    std::atomic<uint32_t> messages{0};
    std::atomic<bool> bad_text{false};
};

TEST_F(RustModule, ValidateBeforeAllocation)
{
    auto bytes = read("cooperative");
    ASSERT_FALSE(bytes.empty());
    Module::Error error;
    auto reject = [&](Module::Limits limits, Module::Error expected) {
        auto *module = Module::load(bytes.data(), bytes.size(), "invalid", limits,
                                    {
                                        this, text
                                    }, error);
        EXPECT_EQ(module, nullptr);
        EXPECT_EQ(error, expected);
        EXPECT_TRUE(Module::release(module));
    };
    reject({0, 131072, 65536}, Module::Error::Limits);
    reject({32768, 4095, 65536}, Module::Error::Limits);
    reject({32768, UINT32_MAX, 65536}, Module::Error::Limits);
    reject({32768, 131072, 16384}, Module::Error::Limits);
    reject({32768, 131072, 20480}, Module::Error::Image); // declared 8192 plus host margin
    bytes.back() ^= 1;
    reject({32768, 131072, 65536}, Module::Error::Image);
}

TEST_F(RustModule, ReleaseWithoutStarting)
{
    modules[0] = load("cooperative");
    ASSERT_NE(modules[0], nullptr);
    EXPECT_EQ(modules[0]->state(), Module::State::Prepared);
    EXPECT_EQ(modules[0]->stats().stack_reserved, 0U);
    modules[0]->request_stop();
    EXPECT_FALSE(modules[0]->start());
    EXPECT_TRUE(Module::release(modules[0]));
    EXPECT_EQ(modules[0], nullptr);
}

TEST_F(RustModule, RawROMFSSourceUsesCopiedExecutableMapping)
{
    modules[0] = load("return", 32768, Module::Source::ROMFS);
    ASSERT_NE(modules[0], nullptr);
    const auto stats = modules[0]->stats();
    EXPECT_GT(stats.rx_reserved, 4096U);
    EXPECT_EQ(stats.rw_reserved, 0U);
    ASSERT_TRUE(modules[0]->start());
    ASSERT_TRUE(join(modules[0]));
    EXPECT_EQ(modules[0]->result(), AP_MODULE_STATUS_OK);
    EXPECT_TRUE(Module::release(modules[0]));
}

TEST_F(RustModule, TwoModulesStopAndDropBeforeRelease)
{
    for (auto &module : modules) {
        module = load("cooperative");
        ASSERT_NE(module, nullptr);
        ASSERT_TRUE(module->start());
    }
    ASSERT_TRUE(ready(2));
    EXPECT_EQ(modules[0]->checksum(), modules[1]->checksum());
    for (auto &module : modules) {
        EXPECT_FALSE(Module::release(module));
        const auto stats = module->stats();
        EXPECT_GT(stats.heap.used, 512U);
        EXPECT_GT(stats.rx_reserved, 4096U);
        EXPECT_EQ(stats.stack_reserved, 65536U);
        EXPECT_GT(stats.thread_reserved, stats.stack_reserved);
        EXPECT_GT(stats.metadata, 0U);
        module->request_stop();
    }
    for (auto &module : modules) {
        ASSERT_TRUE(join(module));
        EXPECT_EQ(module->result(), AP_MODULE_STATUS_STOPPED);
        EXPECT_EQ(module->state(), Module::State::Joined);
        EXPECT_EQ(module->stats().heap.used, 0U);
        EXPECT_EQ(module->stats().thread_reserved, 0U);
        EXPECT_FALSE(module->start());
        EXPECT_TRUE(Module::release(module));
    }
}

TEST_F(RustModule, RepeatedNormalReturn)
{
    for (unsigned i = 0; i < 100; i++) {
        modules[0] = load("return");
        ASSERT_NE(modules[0], nullptr);
        ASSERT_TRUE(modules[0]->start());
        ASSERT_TRUE(join(modules[0]));
        EXPECT_EQ(modules[0]->result(), AP_MODULE_STATUS_OK);
        EXPECT_EQ(modules[0]->stats().heap.used, 0U);
        EXPECT_TRUE(Module::release(modules[0]));
    }
    EXPECT_EQ(messages.load(), 100U);
}

TEST_F(RustModule, FallibleAllocationFailureReturns)
{
    modules[0] = load("cooperative", 256);
    ASSERT_NE(modules[0], nullptr);
    ASSERT_TRUE(modules[0]->start());
    ASSERT_TRUE(join(modules[0]));
    EXPECT_EQ(modules[0]->result(), AP_MODULE_STATUS_NO_MEMORY);
    EXPECT_EQ(modules[0]->stats().heap.used, 0U);
}

TEST_F(RustModule, CleanupMayAllocateAfterStop)
{
    for (unsigned i = 0; i < 100; i++) {
        modules[0] = load("cleanup");
        ASSERT_NE(modules[0], nullptr);
        ASSERT_TRUE(modules[0]->start());
        ASSERT_TRUE(ready(i + 1));
        modules[0]->request_stop();
        ASSERT_TRUE(join(modules[0]));
        EXPECT_EQ(modules[0]->result(), AP_MODULE_STATUS_STOPPED);
        EXPECT_EQ(modules[0]->stats().heap.used, 0U);
        EXPECT_TRUE(Module::release(modules[0]));
    }
}

TEST_F(RustModule, ReleaseReclaimsForgottenAllocations)
{
    for (unsigned i = 0; i < 100; i++) {
        modules[0] = load("leak");
        ASSERT_NE(modules[0], nullptr);
        ASSERT_TRUE(modules[0]->start());
        ASSERT_TRUE(join(modules[0]));
        EXPECT_EQ(modules[0]->result(), AP_MODULE_STATUS_OK);
        EXPECT_GT(modules[0]->stats().heap.used, 512U);
        EXPECT_TRUE(Module::release(modules[0]));
    }
}

TEST_F(RustModule, RetainUntilLateExit)
{
    modules[0] = load("late");
    ASSERT_NE(modules[0], nullptr);
    ASSERT_TRUE(modules[0]->start());
    ASSERT_TRUE(ready(1));
    auto *identity = modules[0];
    modules[0]->request_stop();
    EXPECT_FALSE(Module::release(modules[0]));
    EXPECT_EQ(modules[0], identity);
    EXPECT_GT(modules[0]->stats().heap.used, 0U);
    ASSERT_TRUE(join(modules[0]));
    EXPECT_EQ(modules[0]->result(), AP_MODULE_STATUS_OK);
    EXPECT_EQ(modules[0]->stats().heap.used, 0U);
    EXPECT_TRUE(Module::release(modules[0]));
}

TEST_F(RustModule, PanicRetainsResourcesWithoutBusyLoop)
{
    ASSERT_EXIT(panic_child("panic"), ::testing::ExitedWithCode(73), "");
}

TEST_F(RustModule, InfallibleOOMRetainsResourcesWithoutBusyLoop)
{
    ASSERT_EXIT(panic_child("oom"), ::testing::ExitedWithCode(73), "");
}

TEST_F(RustModule, NoncooperatingWorkerCannotBeReleased)
{
    ASSERT_EXIT({
        auto *module = load("never");
        if (module == nullptr || !module->start() || !ready(1))
        {
            _exit(1);
        }
        module->request_stop();
        const auto *identity = module;
        for (unsigned i = 0; i < 100; i++)
        {
            if (Module::release(module) || module != identity || module->stats().heap.used == 0) {
                _exit(2);
            }
            usleep(1000);
        }
        _exit(73);
    }, ::testing::ExitedWithCode(73), "");
}

TEST_F(RustModule, RetryAfterThreadAllocationFailure)
{
    ASSERT_EXIT({
        auto *module = load("return");
        struct rlimit original;
        if (module == nullptr || getrlimit(RLIMIT_AS, &original) != 0)
        {
            _exit(1);
        }
        auto limited = original;
        limited.rlim_cur = 0; // refuse new stack mappings in this child only
        if (setrlimit(RLIMIT_AS, &limited) != 0)
        {
            _exit(2);
        }
        const bool failed = !module->start();
        if (setrlimit(RLIMIT_AS, &original) != 0)
        {
            _exit(3);
        }
        if (!failed || module->state() != Module::State::Prepared || module->stats().thread_reserved != 0 ||
            !module->start() || !join(module) || module->result() != AP_MODULE_STATUS_OK || !Module::release(module))
        {
            _exit(4);
        }
        _exit(73);
    }, ::testing::ExitedWithCode(73), "");
}

#else
TEST(RustModule, RequiresEnabledLinuxBuild)
{
    GTEST_SKIP() << "Requires enabled native Linux HAL build; exercise SITL sleeps with a simulated vehicle";
}
#endif

AP_GTEST_MAIN()
