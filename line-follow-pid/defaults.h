const int MIN_BASE_SPEED = 40; //100;
const int MAX_BASE_SPEED = 127;
const int START_STOP_COUNT = 2+6; // Start+Stop + Number of crossings
const int SLOWDOWN_SPEED_RATIO = 2; // Half speed

// PID values
const float LOOP_INTERVAL = 0.003;  // 3mS
const float LOOP_FREQUENCY = 1/LOOP_INTERVAL;  // Hz
#define MAX_MOTOR_VOLTS (basespeed > MAX_BASE_SPEED ? MAX_BASE_SPEED : basespeed)
const float PID_Kp = 0.8; //0.5; //0.25; //0.5;
const float PID_Ki = 0.0;
const float PID_Kd = 0.02; //0.02; //0.005; //0.02;

// Marker thresholds
const int markerLowThreshold = 150;
const int markerHighThreshold = markerLowThreshold + 20;
const int lineThreshold = 400;

const int sensorthreshold = (markerLowThreshold + markerHighThreshold)/2; // LED illumination threshold
