#pragma once

//#define SERIAL_DEBUG_PORT 
//#define LOG_RAW_SENSORS

static const float kp = 0.10;
static const float kd = 0.015;

static const float wall_follow_kp = 0.012;
static const float wall_follow_kd = 0.001;

static const float sensor_calibrate_l = 0.29;
static const float sensor_calibrate_f = 0.40;
static const float sensor_calibrate_r = 0.29;

static const float encode_calibrate_l = -1.510; // smaller = bigger turns
static const float encode_calibrate_r = 1.500;

static const float motor_compensation_left = 1.00;  // When motor goes in one direction, it tends to go faster than the other
static const float motor_compensation_right = 0.983;

static const float turn_left_angle_inertia_compensation = 0.8; // End the turns this much short, to allow for inertia taking it the rest of the way
static const float turn_right_angle_inertia_compensation = 0.70; 
static const float turn_right180_angle_inertia_compensation = 0.95;

static const float turning_diameter_mm = 71.0;  // Bigger means turns more

static const float wall_sensor_filter_ratio = 0.1;  // average = old * (1.0-ratio) + new * ratio

static const int forward_speed = 64;
static const int turn_leadin_speed = 48;
static const int turn_speed = 32;
static const int turn_leadout_speed = 48;
static const int turn_180_speed = 32;

// Distance parameters during wall follower
static const int wall_follow_left_distance = 85; //90;
static const int wall_follow_left_distance_min = 50;
static const int wall_follow_forward_min_distance = 98; //90; //95;
static const int wall_follow_left_gap_threshold = 25;
static const int wall_follow_right_gap_threshold = 25;
static const int wall_follow_ahead_blocked_threshold = 40;

// Distances to move during various manoevres
static const int wall_follow_move_left_initial_forward = 85;

// Sensor sensitivities
static const int sensor_left_min_raw = 7000;
static const int sensor_left_max_raw = 12000;
static const int sensor_frontleft_min_raw = 8000;
static const int sensor_frontleft_max_raw = 55000;
static const int sensor_frontright_min_raw = 8000;
static const int sensor_frontright_max_raw = 55000;
static const int sensor_right_min_raw = 7000;
static const int sensor_right_max_raw = 12000;

// For simple follower
const int wallFollowerTargetDistance = 350; 
const int wallFollowerForwardAvoidDistance = 230; 
const int wallFollowerLeftGapThreshold = 150; 
const int wallFollowerLeftTurnDelay = 40;   // Loops before we turn, to prevent colliding with wall

// General
static const int loop_speed_ms = 2; // Plus ADC conversion times

#ifdef SERIAL_DEBUG_PORT
static UART &DebugPort = Serial1; // i.e. UART0 (pins 0&1)
#else
static UART &DebugPort = Serial;  // i.e. USB serial
#endif

