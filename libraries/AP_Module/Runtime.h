// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "AP_Module_config.h"

#if AP_MODULE_ENABLED
#if !HAL_OWNED_THREADS_ENABLED || !HAL_EXECUTABLE_MEMORY_ENABLED
#error "Native modules require owned threads and executable memory"
#endif
#include "Module.h"
#include <AP_Mission/AP_Mission_config.h>
#include <AP_Param/AP_Param.h>

namespace AP_Module
{

// Vehicle-lifetime service. Only its manager thread owns the Module handles.
class Runtime
{
public:
#if AP_MISSION_ENABLED
    struct MissionCommand {
        uint16_t p1;
        float content_p1;
        float content_p2;
        float content_p3;
        uint32_t time_ms;
    };
#endif

    Runtime();
    CLASS_NO_COPY(Runtime);
    static Runtime *get_singleton()
    {
        return _singleton;
    }
    static const AP_Param::GroupInfo var_info[];
    void init();
    void update();
    bool arming_checks(size_t length, char *message) const;
#if AP_MISSION_ENABLED
    void handle_mission_command(uint16_t p1, float content_p1, float content_p2, float content_p3);
    bool pop_mission_command(MissionCommand &command);
#endif

private:
    static constexpr uint8_t MAX_MODULES = 8;
    static constexpr uint32_t CHECKSUM_MASK = 0x007FFFFF;
    enum class Phase : uint8_t { Disabled, Loading, Running, Stopping, Failed };
    enum class ModuleSource : uint8_t { Writable, ROMFS };
    struct ModuleFile {
        char name[64];
        ModuleSource source;
    };
    struct Slot {
        Runtime *owner;
        Module *module;
        bool running;
        bool panic_reported;
        uint32_t heap_expansion_reported;
        uint8_t heap_failures_reported;
    };
    struct Message {
        uint8_t severity;
        char text[45];
    };

    void manager();
    bool load_generation();
    Module *load_file(const ModuleFile &file, Slot &slot);
    Module *load_bytes(const uint8_t *bytes, uint32_t length, const ModuleFile &file, Slot &slot);
    void poll_modules();
    bool all_released() const;
    void begin_stop(bool reload);
    void fail(const char *reason, bool preserve_error = false);
    void report_memory(Module &module);
    void drain_messages();
    static void text(void *context, uint8_t severity, const char *message, uint32_t length);

    static Runtime *_singleton;
    AP_Int8 _enable;
    AP_Int32 _heap_size;
    AP_Int32 _code_size;
    AP_Int32 _stack_size;
    AP_Int8 _max_modules;
    AP_Int16 _restart;
    AP_Int32 _loaded_required;
    AP_Int32 _running_required;
    AP_Int8 _debug_options;

    Module::Limits _limits{};
    uint8_t _capacity{};
    bool _initialized{};
    int16_t _observed_restart{};
    AP_HAL::Scheduler::OwnedThread *_manager_thread{};
    Slot _slots[MAX_MODULES] {};
    bool _reload_after_stop{};
    uint32_t _stop_start_ms{};

    // Never hold this lock over filesystem access, allocation, join, delay,
    // parameter storage or GCS formatting. Worker text callbacks only copy.
    mutable HAL_Semaphore _sem;
    Phase _phase{Phase::Disabled};
    bool _restart_pending{};
    uint32_t _loaded_checksum{};
    uint32_t _running_checksum{};
    uint32_t _heap_required{};
    char _error[80] {};
    Message _messages[8] {};
    uint8_t _message_head{};
    uint8_t _message_count{};
    uint32_t _messages_dropped{};

#if AP_MISSION_ENABLED
    static constexpr uint8_t MISSION_QUEUE_SIZE = 5;
    HAL_Semaphore _mission_sem;
    MissionCommand _mission_commands[MISSION_QUEUE_SIZE] {};
    uint8_t _mission_head{};
    uint8_t _mission_count{};
#endif
};

}
#endif // AP_MODULE_ENABLED
