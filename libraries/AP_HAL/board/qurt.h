#pragma once

#define HAL_BOARD_NAME "QURT"
#define HAL_CPU_CLASS HAL_CPU_CLASS_1000
#define HAL_MEM_CLASS HAL_MEM_CLASS_1000
#define HAL_STORAGE_SIZE            32768
#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE

// only include if compiling C++ code
#ifdef __cplusplus
#include <AP_HAL_QURT/Semaphores.h>
#define HAL_Semaphore QURT::Semaphore
#define HAL_BinarySemaphore QURT::BinarySemaphore
#endif

#ifndef HAL_HAVE_HARDWARE_DOUBLE
#define HAL_HAVE_HARDWARE_DOUBLE 0
#endif

#ifndef HAL_WITH_EKF_DOUBLE
#define HAL_WITH_EKF_DOUBLE HAL_HAVE_HARDWARE_DOUBLE
#endif

#ifndef HAL_GYROFFT_ENABLED
#define HAL_GYROFFT_ENABLED 0
#endif

/*
  disable features for initial port
 */
#define AP_SCRIPTING_ENABLED 0
#define HAL_HAVE_BOARD_VOLTAGE 0
#define HAL_HAVE_SERVO_VOLTAGE 0
#define HAL_HAVE_SAFETY_SWITCH 0
#define HAL_WITH_MCU_MONITORING 0
#define HAL_USE_QUADSPI 0
#define HAL_LOGGING_ENABLED 0
#define HAL_WITH_DSP 0
#define HAL_CANFD_SUPPORTED 0
#define HAL_NUM_CAN_IFACES 0
#define AP_CRASHDUMP_ENABLED 0
#define HAL_ENABLE_DFU_BOOT 0
#define HAL_PARAM_DEFAULTS_PATH nullptr
#define AP_FILESYSTEM_HAVE_DIRENT_DTYPE 0
#define AP_FILESYSTEM_PARAM_ENABLED 0
#define AP_FILESYSTEM_FILE_READING_ENABLED 0

#define USE_LIBC_REALLOC 1


/*
  bring in missing standard library functions
 */
#include <AP_HAL_QURT/replace.h>
