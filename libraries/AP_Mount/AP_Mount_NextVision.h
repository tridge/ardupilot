/*
  Next Vision mount using serial protocol backend class
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include "AP_Mount_Backend.h"
#include <Filter/Filter.h>

#define  NV_DISABLED               0
#define  NV_ENABLED                1

#define  NV_SET_SYSTEM_MODE        0    /* Set the mode of the camera system */
#define  NV_SET_SENSOR             3    /* Adjust which sensor is used (day/IR) */
#define  NV_SET_IR_POLARITY        8

#define  NV_SENSOR_DAY             0
#define  NV_SENSOR_IR              1
#define  NV_IR_POLARITY_WHITE      0
#define  NV_IR_POLARITY_BLACK      1

#define  NV_IR_NO_COLOR            0
#define  NV_IR_ADD_COLOR           1
#define  NV_ZOOM_OUT_ALL           62       // 62 degress is the FOV for zoomed out completely
#define  NV_ZOOM_IN_ALL            5        // 5 degress is the FOV for zoomed in completely

#define  NV_SET_FOV_CMD               4
#define  NV_SET_GIMBAL_CMD            6
#define  NV_SET_SINGLE_YAW_MODE       9
#define  NV_DO_IR_NUC                 11
#define  NV_SET_IR_COLOR              15 
#define  NV_SET_NORMAL_JOYSTICK       16
#define  NV_PILOT_MODE_ANGLE          26    //Pilot view command
#define  NV_PILOT_VIEW_ANGLE          30    //30 degrees from horizontal
#define  NV_SET_CAMERA_STABILIZATION  50

#define  NV_AA_FPV_MODE               10   /* A combination of features defined by Ascent we call FPV mode */

#define  NV_SET_STOW_MODE             0    /* Modes supported by the Next Vision camera system */
#define  NV_SET_PILOT_MODE            1
#define  NV_SET_HOLD_COORDINATE_MODE  2
#define  NV_SET_OBSERVATION_MODE      3
#define  NV_SET_LOCAL_MODE            4
#define  NV_SET_GLOBAL_MODE           5
#define  NV_SET_GRR_MODE              6
#define  NV_SET_TRACKING_MODE         7
#define  NV_SET_EPR_MODE              8

#define  NV_NO_CMD                 0xFF
#define  NV_TRACK_ON_XY            0
#define  NV_TRACK_DISABLED         4
#define  NV_CHAN_0                 0
#define  NV_CHAN_1                 1

#define  NV_VIDEO_ENABLED          0
#define  NV_CAMERA_ENABLED         1
#define  NV_SNAP_SHOT              1
#define  NV_VIDEO_CH0              0
#define  NV_VIDEO_RECORD           2

#define AP_MOUNT_NEXTVISION_SEARCH_MS  60000    //1 minute, long time to wait

typedef enum NV_CAMERA_MODE
{
    NV_CAMERA_DAY_MODE = 0,
    NV_CAMERA_IR_MODE_BLACK = 1,
    NV_CAMERA_IR_MODE_WHITE = 2,
    NV_CAMERA_MAX_MODE = 3
} NV_CAMERA_MODE;


class AP_Mount_NextVision : public AP_Mount_Backend
{

public:
    // Constructor
    AP_Mount_NextVision(AP_Mount &frontend, AP_Mount_Params &params, uint8_t instance);

    // init - performs any required initialisation for this instance
    void init() override;

    // update mount position - should be called periodically
    void update() override;

    // return true if healthy
    bool healthy() const override;

    // has_pan_control - returns true if this mount can control it's pan (required for multicopters)
    bool has_pan_control() const override;

   void set_observation_mode();

   void handle_v2_extension(const mavlink_message_t &msg)override;

  protected:
    // get attitude as a quaternion.  returns true on success
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:
    void look_down()override;
    void stow()override;
    void do_nuc()override;

    bool supports_ftc() const override{	return true;};

    void set_gimbal_rates(float pan_speed, float tilt_speed) override;
    void set_zoom_rate(float zoom_speed) override;
    bool center_camera(bool follow_camera)override;

    void set_camera_mode();
    void write_gimbal_log();
    void find_gimbal();
    void set_cam_stabilization( bool en);
    void ms_delay(uint16_t del_val);
    void measure_freq(char name[5]);        //For debug
    void set_zoom_FOV(float fov_deg);
    void toggle_camera_action();
    void enable_single_yaw();
    void disable_single_yaw();
    void yaw_follow_enable()override {enable_single_yaw();} //Adhere to Ascent Camera API
    void yaw_follow_disable()override {disable_single_yaw();} //Adhere to Ascent Camera API  
    void payload_take_picture()override;
    void start_record()override;
    void stop_record()override;
    void eo_full_screen()override;
    void ir_full_screen()override;
    void eo_ir()override;
    void ir_eo()override;
    void next_color_pallette()override;
    void start_tracking( int x, int y)override;
    void stop_tracking()override;
    void toggle_heat()override;
    void toggle_detection()override;
    void set_detection(int arg)override;
    void fusion()override;
    void set_derotation(bool en);
    void set_report_frequency(int report, int freq);
    void set_angles(float pan_input, float tilt_input)override;

    enum zoom_state{
        ZOOM_STOP,
        ZOOM_IN,
        ZOOM_OUT,
        ZOOM_NO_CHANGE,
    };

    enum stream_control{
        DISABLED,
        DAY,
        IR,
        FUSION,
        PIP,
        SBS //side by side
    };

    struct BIT_Report { //Built in Test
        bool internal_error;
        bool roll_motor;
        bool pitch_motor;
        bool day_sensor;
        bool ir_sensor;
        bool camera_overheat;
        bool camera_communication;
    } bit_report;

    uint8_t  _current_zoom_state;
    uint8_t  _prev_zoom_state = ZOOM_STOP;
    uint16_t _zoom_level;

    // internal variables
    AP_HAL::UARTDriver *_port;

    bool _initialised;              // true once the driver has been initialised
    bool _init_reports = false;
    uint32_t _last_send;            // system time of last do_mount_control sent to gimbal
    uint8_t _sysid = 1;             // sysid - specified by NextVision Spec
    uint8_t _compid = 190;            // component id - specified by NextVision Spec
    mavlink_channel_t _chan;        // mavlink channel used to communicate with gimbal.  Currently hard-coded to Telem2
    uint8_t _camera_mode = NV_SET_OBSERVATION_MODE;
    uint8_t _sensor_mode = NV_CAMERA_DAY_MODE;
    uint8_t _camera_record_function = NV_VIDEO_ENABLED;    // Indicates whether still picture or video able
    bool    _is_recording = false;

    float trip_temperature;
    float camera_temperature;

    bool _tracking = false;
    bool _moving = false;
    bool _ai = false;
    stream_control _stream = DAY;
};