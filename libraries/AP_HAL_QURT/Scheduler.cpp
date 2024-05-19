#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QURT

#include "AP_HAL_QURT.h"
#include "Scheduler.h"

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <sched.h>
#include <errno.h>
#include <stdio.h>
#include <pthread.h>

#include "UARTDriver.h"
#include "Storage.h"
#include "RCOutput.h"
#include <AP_Scheduler/AP_Scheduler.h>

using namespace QURT;

extern const AP_HAL::HAL& hal;

Scheduler::Scheduler()
{
}

void Scheduler::init()
{
    _main_task_pid = getpid();

    // setup the timer thread - this will call tasks at 1kHz
	pthread_attr_t thread_attr;
	struct sched_param param;

	pthread_attr_init(&thread_attr);
	pthread_attr_setstacksize(&thread_attr, 40960);

	param.sched_priority = APM_TIMER_PRIORITY;
	(void)pthread_attr_setschedparam(&thread_attr, &param);

	pthread_create(&_timer_thread_ctx, &thread_attr, &Scheduler::_timer_thread, this);

    // the UART thread runs at a medium priority
	pthread_attr_init(&thread_attr);
	pthread_attr_setstacksize(&thread_attr, 40960);

	param.sched_priority = APM_UART_PRIORITY;
	(void)pthread_attr_setschedparam(&thread_attr, &param);

	pthread_create(&_uart_thread_ctx, &thread_attr, &Scheduler::_uart_thread, this);

    // the IO thread runs at lower priority
	pthread_attr_init(&thread_attr);
	pthread_attr_setstacksize(&thread_attr, 40960);

	param.sched_priority = APM_IO_PRIORITY;
	(void)pthread_attr_setschedparam(&thread_attr, &param);

	pthread_create(&_io_thread_ctx, &thread_attr, &Scheduler::_io_thread, this);
}

void Scheduler::delay_microseconds(uint16_t usec) 
{
    qurt_timer_sleep(usec);
}

void Scheduler::delay(uint16_t ms)
{
    if (!in_main_thread()) {
        ::printf("ERROR: delay() from timer process\n");
        return;
    }
	uint64_t start = AP_HAL::micros64();
    uint64_t now;
    
    while (((now=AP_HAL::micros64()) - start)/1000 < ms) {
        delay_microseconds(1000);
        if (_min_delay_cb_ms <= ms) {
            call_delay_cb();
        }
    }
}

void Scheduler::register_timer_process(AP_HAL::MemberProc proc) 
{
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }

    if (_num_timer_procs < QURT_SCHEDULER_MAX_TIMER_PROCS) {
        _timer_proc[_num_timer_procs] = proc;
        _num_timer_procs++;
    } else {
        hal.console->printf("Out of timer processes\n");
    }
}

void Scheduler::register_io_process(AP_HAL::MemberProc proc) 
{
    for (uint8_t i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i] == proc) {
            return;
        }
    }

    if (_num_io_procs < QURT_SCHEDULER_MAX_TIMER_PROCS) {
        _io_proc[_num_io_procs] = proc;
        _num_io_procs++;
    } else {
        hal.console->printf("Out of IO processes\n");
    }
}

void Scheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us) 
{
    _failsafe = failsafe;
}

void Scheduler::suspend_timer_procs() 
{
    _timer_suspended = true;
}

void Scheduler::resume_timer_procs() 
{
    _timer_suspended = false;
    if (_timer_event_missed == true) {
        _run_timers(false);
        _timer_event_missed = false;
    }
}

void Scheduler::reboot(bool hold_in_bootloader) 
{
    HAP_PRINTF("**** REBOOT REQUESTED ****");
    qurt_timer_sleep(2000000);
    exit(1);
}

void Scheduler::_run_timers(bool called_from_timer_thread)
{
    if (_in_timer_proc) {
        return;
    }
    _in_timer_proc = true;

    if (!_timer_suspended) {
        // now call the timer based drivers
        for (int i = 0; i < _num_timer_procs; i++) {
            if (_timer_proc[i]) {
                _timer_proc[i]();
            }
        }
    } else if (called_from_timer_thread) {
        _timer_event_missed = true;
    }

    // and the failsafe, if one is setup
    if (_failsafe != nullptr) {
        _failsafe();
    }

    _in_timer_proc = false;
}

extern bool qurt_ran_overtime;

void *Scheduler::_timer_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;

    while (!sched->_hal_initialized) {
        sched->delay_microseconds(1000);
    }
    while (true) {
        sched->delay_microseconds(1000);

        // run registered timers
        sched->_run_timers(true);
    }
    return nullptr;
}

void Scheduler::_run_io(void)
{
    if (_in_io_proc) {
        return;
    }
    _in_io_proc = true;

    if (!_timer_suspended) {
        // now call the IO based drivers
        for (int i = 0; i < _num_io_procs; i++) {
            if (_io_proc[i]) {
                _io_proc[i]();
            }
        }
    }

    _in_io_proc = false;
}

void *Scheduler::_uart_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;

    while (!sched->_hal_initialized) {
        sched->delay_microseconds(1000);
    }
    while (true) {
        sched->delay_microseconds(1000);

        // process any pending serial bytes
        //((UARTDriver *)hal.uartA)->timer_tick();
        for (uint8_t i = 0; i < hal.num_serial; i++) {
            auto *p = hal.serial(i);
            if (p != nullptr) {
                p->_timer_tick();
            }
        }
    }
    return nullptr;
}

void *Scheduler::_io_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;

    while (!sched->_hal_initialized) {
        sched->delay_microseconds(1000);
    }
    while (true) {
        sched->delay_microseconds(1000);

        // run registered IO processes
        sched->_run_io();
    }
    return nullptr;
}

bool Scheduler::in_main_thread() const
{
    return getpid() == _main_task_pid;
}

void Scheduler::set_system_initialized() {
    if (_initialized) {
        AP_HAL::panic("PANIC: scheduler::system_initialized called"
                   "more than once");
    }
    _initialized = true;
}

void Scheduler::hal_initialized(void)
{
    HAP_PRINTF("HAL is initialised");
    _hal_initialized = true;
}
#endif
