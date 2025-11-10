/*
  Viewpro mount backend class
 */
#pragma once

#include "AP_Mount_Backend.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_Mount_Viewpro_AA : public AP_Mount_Backend
{

public:
    // Constructor
    AP_Mount_Viewpro_AA(AP_Mount &frontend, AP_Mount_Params &params, uint8_t instance);

    // init
    void init() override;

    // update mount position
    void update() override;

    // has_pan_control
    bool has_pan_control() const override { return true; }

  protected:
    // get attitude as a quaternion.  returns true on success
    bool get_attitude_quaternion(Quaternion& att_quat) override;

  private:
    // internal variables
    AP_HAL::UARTDriver *_port;
    const float rad_to_viewpro_bit = 32768/M_PI;

    bool _initialized;
    bool _following = true; //Need to init as true so it disables it on boot. For some reason it starts in its single yaw mode
    bool _just_centered = false;
    float max_gimbal_speed = 60.0; //In degrees per second
    uint16_t max_zoom_speed = 5; //unitless measure of zoom speed from 1-7 per ViewPro documentation (reduced from float to int in zoom method)
    float _current_zoom = 1.0;
    bool _tracking = false;
    uint8_t _pip_state = 0; //For use with legacy cameras (i.e. before protocol 3.3)
    uint8_t current_color = 0; //For use with legacy cameras (i.e. before protocol 3.3)
    bool _moving = false; //For older protocol gimbals i.e. q10f, z40tir
    uint16_t zoom_in = 0x09;
    uint16_t zoom_out = 0x08;
    uint16_t zoom_stop = 0x01;
    uint16_t zoom_state = zoom_stop;
    uint16_t prev_zoom_state = zoom_stop;
    float fov_angle_h = 53.2; //spec
    float fov_angle_v = 39.8; //spec

    const char* camera_models[10] = {"none","Q10F","Q10T","Z10TIR","Z40K","Z40TIR", "H30T", "Z10TIR Mini","NightHawk","DragonEye"};
    void read_incoming();
    void parse_body();
    void parse_sd_card_reply();
    void write_gimbal_log();
    void update_10hz() override;
    void send_heartbeat();
    void update_vehicle_position();
    bool healthy() const override;
    float get_zoom_level()override;
    
    void payload_take_picture() override;

    void start_record() override;

    void stop_record() override;

    void set_gimbal_rates(float des_pan_speed, float tilt_speed) override;
    void set_zoom_rate(float zoom_speed) override;

    bool center_camera(bool follow_camera) override;

    //Open full screen EO
    void eo_full_screen() override;

    //Open full screen IR
    void ir_full_screen() override;

    //Open full screen EO with IR pip
    void eo_ir() override;

    //Open full screen IR with EO pip
    void ir_eo() override;

    //IR Digital zoom x1
    void digital_zoom_1() override;

    //IR Digital zoom x2
    void digital_zoom_2() override;

    //IR Digital zoom x4
    void digital_zoom_4() override;

    //IR Digital zoom x8
    void digital_zoom_8() override;

    //IR Digital zoom x16
    void digital_zoom_16() override;

    //Start tracking pixel coordinate
    void start_tracking(int x, int y) override;

    //Stop tracking pixel coordinate
    void stop_tracking() override;

    //Iterate through available IR color pallettes
    void next_color_pallette() override;

    //Enable gimbal yaw following (FPV mode)
    void yaw_follow_enable() override;
    
    //Disable gimbal yaw following (FPV mode)
    void yaw_follow_disable() override;

    bool supports_ftc() const override;

    //Look down (tilt camera 90 degrees)
    void look_down() override;

    //Stow (tilt camera up to protect on landing)
    void stow() override;

    void toggle_detection() override;

    void disable_eo_dzoom() override;

    void enable_eo_dzoom() override;

    void checksum_and_send(uint8_t* packet, uint32_t size);
    void checksum_and_send_legacy(uint8_t* packet, uint32_t size);

    //A1 (Parameters in Big Endian format)
    struct PACKED servo_control {
        uint8_t header_0            = 0x55;
        uint8_t header_1            = 0xAA;
        uint8_t header_2            = 0xDC;

        uint8_t body_length         = 0x0C; //servo cmd is 9 bytes plus length byte, frame id byte, and checksum byte
        uint8_t frame_id            = 0x1A;

        uint8_t command_id          = 0x0F;
        uint8_t parameter_1_big     = 0x00;
        uint8_t parameter_1_little  = 0x00;
        uint8_t parameter_2_big     = 0x00;
        uint8_t parameter_2_little  = 0x00;
        uint8_t parameter_3_big     = 0x00;
        uint8_t parameter_3_little  = 0x00;
        uint8_t parameter_4_big     = 0x00;
        uint8_t parameter_4_little  = 0x00;
    };

    //C1
    struct PACKED optical_control {
        uint8_t header_0            = 0x55;
        uint8_t header_1            = 0xAA;
        uint8_t header_2            = 0xDC;

        uint8_t body_length         = 0x05; //optical control is 2 bytes plus length byte, frame id byte, and checksum byte
        uint8_t frame_id            = 0x1C;

        //Named "bytes" and not "Parameters" because functions straddle between bytes; refer to documentation
        uint8_t byte_1              = 0x00;
        uint8_t byte_2              = 0x00;
    };

    //C2
    struct PACKED optical_control_ext {
        uint8_t header_0            = 0x55;
        uint8_t header_1            = 0xAA;
        uint8_t header_2            = 0xDC;

        uint8_t body_length         = 0x06; //optical control ext is 3 bytes plus length byte, frame id byte, and checksum byte
        uint8_t frame_id            = 0x2C;

        uint8_t command_id          = 0x00;
        uint8_t parameter_1_big     = 0x00;
        uint8_t parameter_1_little  = 0x00;
    };

    //E1
    struct PACKED tracking_control {
        uint8_t header_0            = 0x55;
        uint8_t header_1            = 0xAA;
        uint8_t header_2            = 0xDC;

        uint8_t body_length         = 0x06; //Tracking control is 3 bytes plus length byte, frame id byte, and checksum byte
        uint8_t frame_id            = 0x1E;

        uint8_t parameter_1         = 0x00; //This also serves as "tracking source" i.e. EO lens or IR lens
        uint8_t command             = 0x00;
        uint8_t parameter_2         = 0x00;
    };

    //E2
    struct PACKED tracking_control_ext {
        uint8_t header_0            = 0x55;
        uint8_t header_1            = 0xAA;
        uint8_t header_2            = 0xDC;

        uint8_t body_length         = 0x08; //Tracking control ext is 5 bytes plus length byte, frame id byte, and checksum byte
        uint8_t frame_id            = 0x2E;

        uint8_t command             = 0x00; //This also serves as "tracking source" i.e. EO lens or IR lens
        uint8_t parameter_1_big     = 0x00;
        uint8_t parameter_1_little  = 0x00;
        uint8_t parameter_2_big     = 0x00;
        uint8_t parameter_2_little  = 0x00;
    };

    //A1 + C1 + E1
    struct PACKED ViewProAPI {
        uint8_t header_0            = 0x55;
        uint8_t header_1            = 0xAA;
        uint8_t header_2            = 0xDC;

        uint8_t body_length         = 0x11;
        uint8_t frame_id            = 0x30;

        uint8_t command_id          = 0x0F;
        uint8_t parameter_1_big     = 0x00;
        uint8_t parameter_1_little  = 0x00;
        uint8_t parameter_2_big     = 0x00;
        uint8_t parameter_2_little  = 0x00;
        uint8_t parameter_3_big     = 0x00;
        uint8_t parameter_3_little  = 0x00;
        uint8_t parameter_4_big     = 0x00;
        uint8_t parameter_4_little  = 0x00;

        uint8_t byte_1              = 0x00;
        uint8_t byte_2              = 0x00;

        uint8_t parameter_1         = 0x00;
        uint8_t command             = 0x00;
        uint8_t parameter_2         = 0x00;
    };

    //A2 + C2 + E2
    struct PACKED ViewProAPI_ext {
        uint8_t header_0            = 0x55;
        uint8_t header_1            = 0xAA;
        uint8_t header_2            = 0xDC;

        uint8_t body_length         = 0x0D;
        uint8_t frame_id            = 0x31;

        uint8_t A2_byte_1           = 0x00;
        uint8_t A2_byte_2           = 0x00;

        uint8_t C2_command           = 0x00;
        uint8_t C2_parameter_1_big   = 0x00;
        uint8_t C2_parameter_1_little= 0x00;

        uint8_t E2_command           = 0x00;
        uint8_t E2_parameter_1_big   = 0x00;
        uint8_t E2_parameter_1_little= 0x00;
        uint8_t E2_parameter_2_big   = 0x00;
        uint8_t E2_parameter_2_little= 0x00;
        
    };

    //T1 + F1 + B1 + D1 Response Packet
    struct PACKED ViewPro_Reply {
        DEFINE_BYTE_ARRAY_METHODS
        uint8_t header_0;
        uint8_t header_1;
        uint8_t header_2;

        uint8_t body_length;
        uint8_t frame_id;

        uint8_t T1_byte_1;
        uint8_t T1_byte_2;
        uint8_t T1_byte_3;
        uint8_t T1_byte_4;
        uint8_t T1_byte_5;
        uint8_t T1_byte_6;
        uint8_t T1_byte_7;
        uint8_t T1_byte_8;
        uint8_t T1_byte_9;
        uint8_t T1_byte_10;
        uint8_t T1_byte_11;
        uint8_t T1_byte_12;
        uint8_t T1_byte_13;
        uint8_t T1_byte_14;
        uint8_t T1_byte_15;
        uint8_t T1_byte_16;
        uint8_t T1_byte_17;
        uint8_t T1_byte_18;
        uint8_t T1_byte_19;
        uint8_t T1_byte_20;
        uint8_t T1_byte_21;
        uint8_t T1_byte_22;

        uint8_t F1_byte_1;

        uint8_t B1_byte_1;
        uint8_t B1_byte_2;
        uint8_t B1_byte_3;
        uint8_t B1_byte_4;
        uint8_t B1_byte_5;
        uint8_t B1_byte_6;

        uint8_t D1_byte_1;
        uint8_t D1_byte_2;
        uint8_t D1_byte_3;
        uint8_t D1_byte_4;
        uint8_t D1_byte_5;
        uint8_t D1_byte_6;
        uint8_t D1_byte_7;
        uint8_t D1_byte_8;
        uint8_t D1_byte_9;
        uint8_t D1_byte_10;
        uint8_t D1_byte_11;
        uint8_t D1_byte_12;

        uint8_t cs;
    }_buffer;

        //T1 + F1 + B1 + D1 Response Packet
    struct PACKED ViewPro_SD_Reply {
        DEFINE_BYTE_ARRAY_METHODS
        uint8_t header_0;
        uint8_t header_1;
        uint8_t header_2;

        uint8_t body_length;
        uint8_t frame_id;

        uint8_t control_command;
        uint8_t byte2;
        uint8_t byte3;
        uint8_t byte4;
        uint8_t byte5;

        uint8_t cs;
    }_sdcard_buffer;

    struct PACKED SD_Card_Status {
        uint8_t status;
        /*
        1       :       is inserted
        2       :       is initializing
        4       :       is read only
        8       :       is formatted
        16      :       is formatting
        32      :       is full
        64      :       is verified
        128     :       is invalid format
        */

        bool recording = false;
        bool error = false;
        uint32_t capacity; //in MB
        uint32_t remaining_pics;
        uint32_t remaining_record_time;

    }_sd_card_status;

    struct PACKED ViewPro_Heartbeat {
        //TO BE FIXED....currently unused
        uint8_t header_0            = 0x55;
        uint8_t header_1            = 0xAA;
        uint8_t header_2            = 0xDC;

        uint8_t body_length         = 0x04;
        uint8_t frame_id            = 0x10;
        uint8_t byte                = 0x00;
    };

    struct PACKED OldViewProAPI { //With CS, 20 Byte packet
        uint8_t header_0            = 0xFF;
        uint8_t header_1            = 0x01;
        uint8_t header_2            = 0x0F;
        uint8_t header_3            = 0x10;

        uint8_t ctrl_md_0           = 0x00; //roll: no control
        uint8_t ctrl_md_1           = 0x01; //pitch: speed mode
        uint8_t ctrl_md_2           = 0x01; //yaw: speed mode 

        uint8_t roll_speed_0        = 0x00;
        uint8_t roll_speed_1        = 0x00;
        uint8_t roll_angle_0        = 0x00;
        uint8_t roll_angle_1        = 0x00;

        uint8_t pitch_speed_0       = 0x00;
        uint8_t pitch_speed_1       = 0x00;
        uint8_t pitch_angle_0       = 0x00;
        uint8_t pitch_angle_1       = 0x00;

        uint8_t yaw_speed_0        = 0x00;
        uint8_t yaw_speed_1        = 0x00;
        uint8_t yaw_angle_0        = 0x00;
        uint8_t yaw_angle_1        = 0x00;
        uint8_t crc                = 0x00;
    };

    struct PACKED OldViewProZoomAPI {
        uint8_t byte1           = 0x55;
        uint8_t byte2           = 0xAA;
        uint8_t byte3           = 0x1C;
        uint8_t byte4           = 0x04;
        uint8_t byte5           = 0x00; 
        uint8_t byte6           = 0x00; 
        uint8_t byte7           = 0x10;
        uint8_t byte8           = 0x00; 
        uint8_t byte9           = 0x00; 
        uint8_t byte10          = 0x30;  
    };

    struct PACKED AHRS_Packet {
        uint8_t header_0            = 0x55;
        uint8_t header_1            = 0xAA;
        uint8_t header_2            = 0xDC;

        uint8_t body_length         = 0x2D;
        uint8_t frame_id            = 0xB1;

        uint8_t data_group                      = 0x07;
        uint8_t res1                            = 0x00;
        uint8_t res2                            = 0x00;
        uint8_t res3                            = 0x00;
        uint8_t res4                            = 0x00;
        uint8_t res5                            = 0x00;
        uint8_t res6                            = 0x00;
        uint8_t res7                            = 0x00;
        uint8_t tilt_angle2                      = 0;
        uint8_t tilt_angle1                      = 0;
        uint8_t roll_angle2                      = 0;
        uint8_t roll_angle1                      = 0;
        uint8_t yaw_angle2                       = 0;
        uint8_t yaw_angle1                       = 0;
        uint8_t date2                           = 0;
        uint8_t date1                           = 0;
        uint8_t time_high                       = 0x00;
        uint8_t time_mid                        = 0x00;
        uint8_t time_low                        = 0x00;
        uint8_t gps_yaw2                         = 0;
        uint8_t gps_yaw1                         = 0;
        uint8_t position_mark                   = 0x00;
        uint8_t lat4                             = 0;
        uint8_t lat3                             = 0;
        uint8_t lat2                             = 0;
        uint8_t lat1                             = 0;
        uint8_t lng4                             = 0;
        uint8_t lng3                             = 0;
        uint8_t lng2                             = 0;
        uint8_t lng1                             = 0;
        uint8_t alt4                             = 0;
        uint8_t alt3                             = 0;
        uint8_t alt2                             = 0;
        uint8_t alt1                             = 0;
        uint8_t ground_lat_speed2                = 0;
        uint8_t ground_lat_speed1                = 0;
        uint8_t ground_long_speed2               = 0;
        uint8_t ground_long_speed1               = 0;
        uint8_t vdop2                           = 0;
        uint8_t vdop1                           = 0;
        uint8_t ground_alt_speed2                = 0;
        uint8_t ground_alt_speed1                = 0;
    };
};