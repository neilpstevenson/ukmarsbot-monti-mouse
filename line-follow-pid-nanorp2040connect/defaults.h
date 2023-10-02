#pragma once

#define SENSOR_POLAIRTY_TRUE 0 // 1 = If WHITE is a higher value (i.e. Neil's compact sensor), otherwise 0 (the UKMARSBOT sensors)

const int MIN_BASE_SPEED = 40; //100;
const int MAX_BASE_SPEED = 127;
const int START_STOP_COUNT_DEFAULT = 2+2; //e.g. 8 for 2023 track (start, stop + no. of crosses), 2 for drag only
const int START_STOP_COUNT_DRAGSTER = 2;
const int SLOWDOWN_SPEED_RATIO = 2; // Half speed

const float ENCODER_CALIBRATION = 1.7;  // Counts to mm

// Tolerances etc. all in mm
const int CROSSOVER_TOLERANCE = 30;
const int STRAIGHT_LINE_TOLERANCE = 45;
const int STOP_DISTANCE = 120 - CROSSOVER_TOLERANCE;

// PID values
const float LOOP_INTERVAL = 0.003;  // 3mS
const float LOOP_FREQUENCY = 1/LOOP_INTERVAL;  // Hz
#define MAX_MOTOR_VOLTS (basespeed > MAX_BASE_SPEED ? MAX_BASE_SPEED : basespeed)
const float PID_Kp = 0.03;
const float PID_Ki = 0.0;
const float PID_Kd = 0.002;

const float PID_Kp_DRAGSTER = 0.1;
const float PID_Ki_DRAGSTER = 0.0;
const float PID_Kd_DRAGSTER = 0.01;

// Marker thresholds
const int markerLowThreshold = 350; //850; //700; // e.g. 250 for old full-size or half-size board, 850 for neil's new inverted board
const int markerHighThreshold = markerLowThreshold+20;

const int sensorthreshold = (markerLowThreshold + markerHighThreshold)/2; // LED illumination threshold
