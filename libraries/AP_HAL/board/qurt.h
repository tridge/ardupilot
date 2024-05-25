#pragma once

#include <AP_HAL_QURT/AP_HAL_QURT_Main.h>

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
#define HAL_WITH_DSP 0
#define HAL_CANFD_SUPPORTED 0
#define HAL_NUM_CAN_IFACES 0
#define AP_CRASHDUMP_ENABLED 0
#define HAL_ENABLE_DFU_BOOT 0
#define AP_FILESYSTEM_HAVE_DIRENT_DTYPE 0
#define HAL_PICCOLO_CAN_ENABLE 0
#define LUA_USE_LONGJMP 1
#define AP_MAVLINK_FTP_ENABLED 0
#define HAL_LOGGING_MAVLINK_ENABLED 0
#define HAL_LOGGING_FILESYSTEM_ENABLED 0
#define AP_FILESYSTEM_POSIX_HAVE_UTIME 0
#define AP_FILESYSTEM_POSIX_HAVE_FSYNC 0
#define AP_FILESYSTEM_POSIX_HAVE_STATFS 0
// Get undefined symbols if this isn't enabled:
//      _ZN7NavEKF39Log_WriteEv
//      _ZNK7AP_AHRS11Write_AHRS2Ev
//      _ZNK7AP_AHRS9Write_POSEv
// #define HAL_LOGGING_ENABLED 0
// #define AP_GRIPPER_ENABLED 0
// #define AP_AIRSPEED_ENABLED 0
// #define AP_SRV_CHANNELS_ENABLED 0
// #define HAL_RUNCAM_ENABLED 0
// #define HAL_HOTT_TELEM_ENABLED 0
// #define HAL_VISUALODOM_ENABLED 0
// #define AP_VIDEOTX_ENABLED 0
// #define AP_SMARTAUDIO_ENABLED 0
// #define AP_TRAMP_ENABLED 0
// #define AP_OPENDRONEID_ENABLED 0
// #define HAL_EFI_ENABLED 0
// #define AP_TEMPERATURE_SENSOR_ENABLED 0
// #define AP_KDECAN_ENABLED 0
// #define AP_AIS_ENABLED 0
// #define HAL_NMEA_OUTPUT_ENABLED 0
// #define AP_FENCE_ENABLED 0
// #define AP_CUSTOMROTATIONS_ENABLED 0
// #define AP_DDS_ENABLED 0
// #define AP_CHECK_FIRMWARE_ENABLED 0
// #define HAL_MSP_ENABLED 0
// #define HAL_EXTERNAL_AHRS_ENABLED 0
// #define HAL_GENERATOR_ENABLED 0
// #define HAL_CANMANAGER_ENABLED 0

// SITL on Hardware definitions
// env SIM_ENABLED 1
//
// #define INS_MAX_INSTANCES 2
// #define HAL_COMPASS_MAX_SENSORS 2
//
// #define AP_GPS_BACKEND_DEFAULT_ENABLED 0
// #define AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED 0
//
// #define HAL_NAVEKF2_AVAILABLE 0
// #define EK3_FEATURE_BODY_ODOM 0
// #define EK3_FEATURE_EXTERNAL_NAV 0
// #define EK3_FEATURE_DRAG_FUSION 0
// #define HAL_ADSB_ENABLED 0
// #define HAL_PROXIMITY_ENABLED 0
// #define HAL_VISUALODOM_ENABLED 0
// #define HAL_GENERATOR_ENABLED 0
//
// #define HAL_MSP_OPTICALFLOW_ENABLED 0
// #define HAL_SUPPORT_RCOUT_SERIAL 0
// #define HAL_HOTT_TELEM_ENABLED 0
// #define HAL_HIGH_LATENCY2 0
//
// #define AP_SIM_INS_FILE_ENABLED 0
// End of SITL on Hardware definitions

#define HAL_PARAM_DEFAULTS_PATH nullptr
#define HAL_BOARD_STORAGE_DIRECTORY "APM"
#define HAL_BOARD_LOG_DIRECTORY "APM/logs"
#define HAL_BOARD_TERRAIN_DIRECTORY "APM/terrain"

#define USE_LIBC_REALLOC 1

/*
  barometer list
 */
#define PROBE_BARO_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_Baro_ ## driver::probe(*this,std::move(GET_I2C_DEVICE(bus, addr)),##args))
#define HAL_BARO_PROBE_LIST PROBE_BARO_I2C(ICP101XX, 3, 0x63)


/*
  bring in missing standard library functions
 */
#include <AP_HAL_QURT/replace.h>
