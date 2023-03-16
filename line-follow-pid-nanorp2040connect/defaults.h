const int MIN_BASE_SPEED = 40; //100;
const int MAX_BASE_SPEED = 127;
const int START_STOP_COUNT_DEFAULT = 4; //4 for track, 2 for drag
const int START_STOP_COUNT_DRAGSTER = 2;
const int SLOWDOWN_SPEED_RATIO = 2; // Half speed

// PID values
const float LOOP_INTERVAL = 0.003;  // 3mS
const float LOOP_FREQUENCY = 1/LOOP_INTERVAL;  // Hz
#define MAX_MOTOR_VOLTS (basespeed > MAX_BASE_SPEED ? MAX_BASE_SPEED : basespeed)
const float PID_Kp = 0.9;
const float PID_Ki = 0.0;
const float PID_Kd = 0.03;

const float PID_Kp_DRAGSTER = 0.1;
const float PID_Ki_DRAGSTER = 0.0;
const float PID_Kd_DRAGSTER = 0.01;

// Marker thresholds
const int markerLowThreshold = 850; //700;
const int markerHighThreshold = markerLowThreshold+20;

const int sensorthreshold = 700; // LED illumination threshold
