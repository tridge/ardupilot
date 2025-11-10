// user defined variables

#ifdef USERHOOK_VARIABLES

#define FTC_YAW_GAIN 5.0

float _aft_rpm;
float _fwd_rpm;

float _fwd_thr;
float _aft_thr;

bool _servo_test1_complete;
bool _servo_test2_complete;
bool _servo_test3_complete;
uint16_t servo_test_start_ms;

uint16_t start_rpm_comp_time;
uint8_t rpm_update_counter;

//Counters and timers
bool killswitch_pressed;
uint16_t killswitch_timer;

// Auto Tune
int8_t num_battery;
float payload_weight;
float vehicle_weight;

// Herelink decoding of buttons
uint32_t ch9_timer;
uint32_t ch10_timer;
uint32_t ch11_timer;

bool ch9_button_pressed;
bool ch10_button_pressed;
bool ch11_button_pressed;

bool long_press_flag_ch9;
bool long_press_flag_ch10;
bool long_press_flag_ch11;

bool short_press_flag_ch9;
bool short_press_flag_ch10;
bool short_press_flag_ch11;

bool ch9_button_hold;
bool ch10_button_hold;
bool ch11_button_hold;

bool manual_zoom_control = false;

int8_t camera_type;

enum vehicle_state{
	disarm,
	spoolup,
	on_ground,
	takeoff,
	hover,
	landing,
	calculating,

} spirit_state;

bool prev_armed = false;

//Used to track state machine for centering camera in AP_Mount classes
bool centering_camera = false;

float mav_gimbal_pan = 0.0;
float mav_gimbal_tilt = 0.0;
bool  gcs_initialized = false;

uint8_t prev_ekf_source;
uint32_t last_ekf_source_switch_ms;

bool mot_offs_calculated = false;
AverageFilterFloat_Size20 mot_x_rest;
AverageFilterFloat_Size20 mot_y_rest;
AverageFilterFloat_Size20 mot_z_rest;
AverageFilterFloat_Size20 curr_rest;
float mot_x_rest_avg;
float mot_y_rest_avg;
float mot_z_rest_avg;
float curr_rest_avg;

AverageFilterFloat_Size20 offs_x_avg_filt;
AverageFilterFloat_Size20 offs_y_avg_filt;
AverageFilterFloat_Size20 offs_z_avg_filt;
float offs_x_avg;
float offs_y_avg;
float offs_z_avg;

int tube_launch = false;
float yaw_p = 0.09;  //Not hardcoded, just for initialization
float yaw_i = 0.015; //Not hardcoded, just for initialization
float yaw_p_ll = 0.04;  //Not hardcoded, just for initialization
float yaw_i_ll = 0.007; //Not hardcoded, just for initialization

uint32_t calc_delay;
Vector3f initial_motor_offsets;
int8_t auto_mot = 1;

#endif  // USERHOOK_VARIABLES