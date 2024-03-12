#pragma once

#define SENSOR_POLAIRTY_TRUE 0 // 1 = If WHITE is a higher value (i.e. Neil's compact sensor), otherwise 0 (the UKMARSBOT sensors)

const int MIN_BASE_SPEED = 40; //100;
//const int MAX_BASE_SPEED = 170;
const float SLOWDOWN_SPEED_RATIO = 0.7; // For segment after the end marker
//const int FAST_FORWARD_SPEEDUP = 30; // Added to forward speed during straights
const int ILLUMINATION_ON_TIME_uS = 100;

const float ENCODER_CALIBRATION = 1.7;  // Counts to mm

static const float motor_compensation_left = 1.00;  // When motor goes in one direction, it tends to go faster than the other
static const float motor_compensation_right = 0.983;

// Tolerances etc. all in mm
const int CROSSOVER_TOLERANCE = 30;
const int STRAIGHT_LINE_TOLERANCE = 4;
const int SEGMENT_START_DEFERRED_DISTANCE = 30; // We get this much into a segment before recording the distance (allows for straightening up etc)

// PID max result
#define MAX_MOTOR_VOLTS 1

// Loop values
const float LOOP_INTERVAL = 0.003;  // 3mS
const float LOOP_FREQUENCY = 1/LOOP_INTERVAL;  // Hz

// Marker thresholds
const int markerLowThreshold = 740; //250; //850; //700; // e.g. 740 for white 1/2 size board, 250 for old full-size or half-size board, 850 for neil's new inverted board
const int markerHighThreshold = markerLowThreshold+20;

const int sensorthreshold = (markerLowThreshold + markerHighThreshold)/2; // LED illumination threshold

// Support methods
extern void buttonwait(int period);
extern void photoread(bool polarity = SENSOR_POLAIRTY_TRUE);
extern void logSensors(const char *mode);
extern void functionswitch();
extern void stopmotors();
extern bool batterycheck();

extern int rightspeed;
extern int leftspeed;

extern int lfrontsens;
extern int lsidesens;
extern int rsidesens;
extern int rfrontsens;
extern int sensdiff;

extern int fnswvalue;
