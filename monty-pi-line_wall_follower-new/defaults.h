#pragma once

#define SERIAL_DEBUG_PORT 
//#define LOG_RAW_SENSORS

const int MIN_BASE_SPEED = 100;
//const int MAX_BASE_SPEED = 170;
const float SLOWDOWN_SPEED_RATIO = 0.7; // For segment after the end marker
//const int FAST_FORWARD_SPEEDUP = 30; // Added to forward speed during straights
const int ILLUMINATION_ON_TIME_uS = 100;

const float ENCODER_CALIBRATION = 1.7;  // Counts to mm

static const float motor_compensation_left = 1.00;  // When motor goes in one direction, it tends to go faster than the other
static const float motor_compensation_right = 1.00; //0.983;

// Tolerances etc. all in mm
const int CROSSOVER_TOLERANCE = 30;
const int STRAIGHT_LINE_TOLERANCE = 10;
const int MIN_RADIUS_FLAT_OUT = 220;  // Go at full speed above this radius
const int SEGMENT_START_DEFERRED_DISTANCE = 30; // We get this much into a segment before recording the distance (allows for straightening up etc)

// PID max result
//#define MAX_MOTOR_VOLTS 1

// Loop values
//const float LOOP_INTERVAL = 0.003;  // 3mS
//const float LOOP_FREQUENCY = 1/LOOP_INTERVAL;  // Hz

// Marker thresholds
const int defaultMarkerLowThreshold = 740; //250; //850; //700; // e.g. 740 for white 1/2 size board, 250 for old full-size or half-size board, 850 for neil's new inverted board
extern int markerLowThreshold;
//const int markerHighThreshold = markerLowThreshold+20;

//const int sensorthreshold = (markerLowThreshold + markerHighThreshold)/2; // LED illumination threshold

// Support methods
//extern bool buttonPressed();
extern void buttonwait(int period);
//extern void photoread(bool polarity = SENSOR_POLAIRTY_TRUE);
//extern void logSensors(const char *mode);
//extern void functionswitch();
//extern void stopmotors();
extern bool batterycheck();

extern int rightspeed;
extern int leftspeed;

//extern int lfrontsens;
//extern int lsidesens;
//extern int rsidesens;
//extern int rfrontsens;
extern int sensdiff;

//extern int fnswvalue;

//#ifdef SERIAL_DEBUG_PORT
//static UART &DebugPort = Serial1; // i.e. UART0 (pins 0&1)
//#else
//static UART &DebugPort = Serial;  // i.e. USB serial
//#endif
