const int MIN_BASE_SPEED = 40; //100;
const int MAX_BASE_SPEED = 100;
const int START_STOP_COUNT = 4;
const int SLOWDOWN_SPEED_RATIO = 2; // Half speed

// PID values
const float LOOP_INTERVAL = 0.003;  // 1mS
const float LOOP_FREQUENCY = 1/LOOP_INTERVAL;  // Hz
#define MAX_MOTOR_VOLTS (basespeed > MAX_BASE_SPEED ? MAX_BASE_SPEED : basespeed)
const float PID_Kp = 0.6;
const float PID_Ki = 0.0;
const float PID_Kd = 0.15;

// Marker thresholds
const int markerLowThreshold = 100;
const int markerHighThreshold = 300;

const int sensorthreshold = 500; // LED illumination threshold
