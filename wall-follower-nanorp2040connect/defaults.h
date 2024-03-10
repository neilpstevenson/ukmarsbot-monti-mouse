#pragma once

#define SENSOR_POLAIRTY_TRUE 1 // 1 = If WHITE is a higher value (i.e. Neil's compact sensor), otherwise 0 (the UKMARSBOT sensors)

const int MIN_BASE_SPEED = 40; //100;
const int MAX_BASE_SPEED = 127;
const float SLOWDOWN_SPEED_RATIO = 0.7; // For segment after the end marker
//const int FAST_FORWARD_SPEEDUP = 30; // Added to forward speed during straights
const int ILLUMINATION_ON_TIME_uS = 100;

const float ENCODER_CALIBRATION = 1.7;  // Counts to mm

// Tolerances etc. all in mm
const int CROSSOVER_TOLERANCE = 30;
const int STRAIGHT_LINE_TOLERANCE = 4;
const int SEGMENT_START_DEFERRED_DISTANCE = 30; // We get this much into a segment before recording the distance (allows for straightening up etc)
const int ACCELERATION_DISTANCE = 60;
const int DECELERATION_DISTANCE = 150;
const float CORNER_APPROACH_DISTANCE = 1.0; // Distance in mm * speed
const int STOP_DISTANCE = 80 - CROSSOVER_TOLERANCE;

// PID values
const float LOOP_INTERVAL = 0.003;  // 3mS
const float LOOP_FREQUENCY = 1/LOOP_INTERVAL;  // Hz
#define MAX_MOTOR_VOLTS 1
const float PID_Kp = 0.012; // 0.015;    // 0.03 old 1/2 size board
const float PID_Ki = 0.0;
const float PID_Kd = 0.001; // 0.001;  // 0.0025 old 1/2 size board

// Marker thresholds
const int markerLowThreshold = 100; //740; //250; //850; //700; // e.g. 740 for white 1/2 size board, 250 for old full-size or half-size board, 850 for neil's new inverted board
const int markerHighThreshold = markerLowThreshold+20;

const int sensorthreshold = (markerLowThreshold + markerHighThreshold)/2; // LED illumination threshold

const int wallFollowerTargetDistance = 100; 
const int wallFollowerForwardAvoidDistance = 200; 
