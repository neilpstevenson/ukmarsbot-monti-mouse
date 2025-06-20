/******************************************************************************
 * Project: mazerunner-core                                                   *
 * -----                                                                      *
 * Copyright 2022 - 2023 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/
#pragma once

#include <Arduino.h>
#include <WiFiNINA.h> // RP2040 Connect for definition of A6/A7 etc.

/*****************************************************************************
 *
 * HALF MONTY is a half-size mouse based on an Waveshare RP2040 Zero board
 *
 * It uses a wall sensor board with four emitter-detector pairs. Gearmotors
 * with 20:1 ratio gearboxes and encoder discs with 6 magnets in each.
 *
 * The sensors consist of SFH4550 emitters and SFH309FA detectors.
 *
 *****************************************************************************/
#define NAME "MONTY PI"

//***** SENSOR CALIBRATION **************************************************//
/**
RAW side sensor values when robot is centred in a cell and no wall ahead

##                            ##
||                            ||
||                            ||
||          ______            ||
||         /       \          ||
||         |       |          ||
||         |||   |||          ||
||         |       |          ||
||         |_______|          ||
##============================##

RAW values for the front sensor when the robot is backed up to a wall

##=============================##


             ______
           /       \
           |       |
           |||   |||
           |       |
           |_______|
##=============================##
*/

#if EVENT == EVENT_HOME
// wall sensor thresholds and constants
// RAW values for the front sensor when the robot is backed up to a wall
// with another wall ahead
const int FRONT_LEFT_CALIBRATION = 1500; //790;
const int FRONT_RIGHT_CALIBRATION = 1500;
// RAW values for the side sensors when the robot is centered in a cell
// and there is no wall ahead
const int LEFT_CALIBRATION = 3000;;
const int RIGHT_CALIBRATION = 3000;

// The front linear constant is the value of k needed to make the function
// sensors.get_distance(sensor,k) return 68mm (half=30mm) when the mouse is backed up
// against a wall with only a wall ahead and sensor is the calibrated sum of the front sensors
const int FRONT_LINEAR_CONSTANT = 961; //i.e = sqrt(sum(FL,FR)) * 68 = sqrt(200) * 68
const int FRONT_REFERENCE = 500;  // sum reading when mouse centered with wall ahead

// SS90E turn thresholds. This is the front sum reading to trigger a turn
// it changes a bit if there is an adjacent wall. The threshold is set for
// when the robot is 20mm past the cell boundary. That is, the distance
// from the front of the mouse to the wall ahead is 92mm (half=50mm)
const int TURN_THRESHOLD_SS90E = 108; //124;
const int EXTRA_WALL_ADJUST = 10;

// Threshold used for starting the robot runs
const int OCCLUDED_THRESHOLD_FRONT_RAW = 900;

#elif EVENT == EVENT_UK
// wall sensor thresholds and constants
// RAW values for the front sensor when the robot is backed up to a wall
// with another wall ahead
const int FRONT_LEFT_CALIBRATION = 1210; //790;
const int FRONT_RIGHT_CALIBRATION = FRONT_LEFT_CALIBRATION;  // Same sensor
// RAW values for the side sensors when the robot is centered in a cell
// and there is no wall ahead
const int LEFT_CALIBRATION = 1120; //870;
const int RIGHT_CALIBRATION = 1000; //890;

// The front linear constant is the value of k needed to make the function
// sensors.get_distance(sensor,k) return 68mm (half=30mm) when the mouse is backed up
// against a wall with only a wall ahead and sensor is the calibrated sum of the front sensors
const int FRONT_LINEAR_CONSTANT = 961; //i.e = sqrt(sum(FL,FR)) * 68 = sqrt(200) * 68
const int FRONT_REFERENCE = 500;  // sum reading when mouse centered with wall ahead

// SS90E turn thresholds. This is the front sum reading to trigger a turn
// it changes a bit if there is an adjacent wall. The threshold is set for
// when the robot is 20mm past the cell boundary. That is, the distance
// from the front of the mouse to the wall ahead is 92mm (half=50mm)
const int TURN_THRESHOLD_SS90E = 108; //124;
const int EXTRA_WALL_ADJUST = 10;

// Threshold used for starting the robot runs
const int OCCLUDED_THRESHOLD_FRONT_RAW = 900;

#endif

//***** IO PINS *****************************************************//
// the BASIC sensor board has two LEDs
// const int LED_LEFT = USER_IO;
// const int LED_RIGHT = EMITTER_A;
// const int LED_USER = USER_IO;
// but only one emitter pin
// const int EMITTER_FRONT = EMITTER_B;
// const int EMITTER_DIAGONAL = EMITTER_B;

// the ADVANCED sensor board has only one LED so use the value twice
const int LED_LEFT = LED_LEFT_IO;
const int LED_RIGHT = LED_RIGHT_IO;
//const int LED_USER = LED_NEOPIXEL_IO; // TODO
// but two emitter pins
const int EMITTER_FRONT = EMITTER_A;
const int EMITTER_DIAGONAL = EMITTER_B;

//***** SENSOR HARDWARE *****************************************************//
// the ADC channels corresponding to the sensor inputs. There are 8 available
// Channels 0..3 are normally used for sensors.
// Channels 4 and 5 are available if you do not want to add an I2C device
// Channel 6 is pre-allocated to the Battery monitor
// Channel 7 is re-allocated to the function switch and button

// NOTE - these are the AnalogueConverter channel indexes, not necessariy the
// hardware ADC channel numbers

// ADVANCED SENSOR
//const int RFS_ADC_CHANNEL = 1;
//const int RSS_ADC_CHANNEL = 3;
//const int LSS_ADC_CHANNEL = 0;
//const int LFS_ADC_CHANNEL = 2;

// BASIC SENSOR - just repeat the front sensor to make the code cleaner
// const int RFS_ADC_CHANNEL = 1;
// const int RSS_ADC_CHANNEL = 0;
// const int LSS_ADC_CHANNEL = 2;
// const int LFS_ADC_CHANNEL = 1;

// Line Follower Sensor - just repeat the front sensor to make the code cleaner
const int RFS_ADC_CHANNEL = 1;
const int RSS_ADC_CHANNEL = 0;
const int LSS_ADC_CHANNEL = 3;
const int LFS_ADC_CHANNEL = 2;

#define SENSORS_INVERTED_POLARITY 1
#define SENSOR_POLAIRTY_TRUE 1 // 1 = If WHITE is a higher value (i.e. Neil's compact sensor), otherwise 0 (the UKMARSBOT sensors)

// there are two other ADC channels used by the robot
const int SWITCHES_ADC_CHANNEL = 6;
const int BATTERY_ADC_CHANNEL = 7;
//***************************************************************************//
const uint32_t BAUDRATE = 115200;

//***************************************************************************//
// set this to zero to disable profile data logging over SerialPort
#define DEBUG_LOGGING 1
// time between logged lines when reporting is enabled (milliseconds)
const int REPORTING_INTERVAL = 10;

//***************************************************************************//
// Some physical constants that are likely to be robot-specific
// with robot against back wall, how much travel is there to the cell center?
const int BACK_WALL_TO_CENTER = 48;

//***************************************************************************//
// We need to know about the drive mechanics.
// The encoder pulse counts should be obvious from the encoder itself.
// Work out the gear ratio by rotating the wheel a number of turns and counting
// the pulses.
// Finally, move the mouse in a straight line through 1000mm of travel to work
// out the wheel diameter.
const float ENCODER_PULSES = 12.00;
const float GEAR_RATIO = 11.0;
const float WHEEL_DIAMETER = 31.25;

// Mouse radius is the distance between the contact patches of the drive wheels.
// A good starting approximation is half the distance between the wheel centres.
// After testing, you may find the working value to be larger or smaller by some
// small amount. AFTER you have the wheel diameter and gear ratio calibrated,
// have the mouse turn in place and adjust the MOUSE_RADIUS until these turns are
// as accurate as you can get them
const float MOUSE_RADIUS = 40.4; // Adjust on test - bigger for more turn

// The robot is likely to have wheels of different diameters or motors of slightly
// different characteristics and that must be compensated for if the robot is to
// reliably drive in a straight line.
// This number adjusts the encoder count and must be  added to the right
// and subtracted from the left motor.
const float ROTATION_BIAS = -0.003; // Negative makes robot curve to left

// Now we can pre-calculate the key constats for the motion control
const float MM_PER_COUNT = PI * WHEEL_DIAMETER / (ENCODER_PULSES * GEAR_RATIO);
const float MM_PER_COUNT_LEFT = (1 - ROTATION_BIAS) * MM_PER_COUNT;
const float MM_PER_COUNT_RIGHT = (1 + ROTATION_BIAS) * MM_PER_COUNT;
const float DEG_PER_MM_DIFFERENCE = (180.0 / (2 * MOUSE_RADIUS * PI));

//*** MOTION CONTROLLER CONSTANTS **********************************************//

//***************************************************************************//
// Control loop timing. Pre-calculate to save time in interrupts
const float LOOP_FREQUENCY = 500.0;
const float LOOP_INTERVAL = (1.0 / LOOP_FREQUENCY);

// Dynamic performance constants
// There is a video describing how to get these numbers and calculate the feedforward
// constnats here: https://youtu.be/BrabDeHGsa0
const float FWD_KM = 1200; //475.0;  // mm/s/Volt
const float FWD_TM = 0.190;  // forward time constant
const float ROT_KM = 1500; //775.0;  // deg/s/Volt
const float ROT_TM = 0.210;  // rotation time constant

// Motor Feedforward
/***
 * Speed Feedforward is used to add a drive voltage proportional to the motor speed
 * The units are Volts per mm/s and the value will be different for each
 * robot where the motor + gearbox + wheel diamter + robot weight are different
 * You can experimentally determine a suitable value by turning off the controller
 * and then commanding a set voltage to the motors. The same voltage is applied to
 * each motor. Have the robot report its speed regularly or have it measure
 * its steady state speed after a period of acceleration.
 * Do this for several applied voltages from 0.5 Volts to 5 Volts in steps of 0.5V
 * Plot a chart of steady state speed against voltage. The slope of that graph is
 * the speed feedforward, SPEED_FF.
 * Note that the line will not pass through the origin because there will be
 * some minimum voltage needed just to ovecome friction and get the wheels to turn at all.
 * That minimum voltage is the BIAS_FF. It is not dependent upon speed but is expressed
 * here as a fraction for comparison.
 */
const float MAX_MOTOR_VOLTS = 7.0;

const float SPEED_FF = (1.0 / FWD_KM);
const float ACC_FF = (FWD_TM / FWD_KM);
const float BIAS_FF = 0.121;
const float TOP_SPEED = (6.0 - BIAS_FF) / SPEED_FF;

//*** MOTION CONTROL CONSTANTS **********************************************//

// forward motion controller constants
const float FWD_ZETA = 0.707;
const float FWD_TD = FWD_TM;

const float FWD_KP = 16 * FWD_TM / (FWD_KM * FWD_ZETA * FWD_ZETA * FWD_TD * FWD_TD);
const float FWD_KD = LOOP_FREQUENCY * (8 * FWD_TM - FWD_TD) / (FWD_KM * FWD_TD);

// rotation motion controller constants
const float ROT_ZETA = 0.707;
const float ROT_TD = ROT_TM;

const float ROT_KP = 16 * ROT_TM / (ROT_KM * ROT_ZETA * ROT_ZETA * ROT_TD * ROT_TD);
const float ROT_KD = LOOP_FREQUENCY * (8 * ROT_TM - ROT_TD) / (ROT_KM * ROT_TD);

// controller constants for the steering controller
const float STEERING_KP = 0.01;
const float STEERING_KD = 0.0008; //0.001;
const float STEERING_ADJUST_LIMIT = 30.0;  // deg/s

// encoder polarity is either 1 or -1 and is used to account for reversal of the encoder phases
#define ENCODER_LEFT_POLARITY (-1)
#define ENCODER_RIGHT_POLARITY (1)

// similarly, the motors may be wired with different polarity and that is defined here so that
// setting a positive voltage always moves the robot forwards
#define MOTOR_LEFT_POLARITY (1)
#define MOTOR_RIGHT_POLARITY (-1)

//***************************************************************************//

//***** PERFORMANCE CONSTANTS************************************************//
// search and run speeds in mm/s and mm
const int SEARCH_SPEED = 400;
const int SEARCH_ACCELERATION = 2000;
const int SEARCH_TURN_SPEED = 300;
const int SMOOTH_TURN_SPEED = 500;
const int FAST_TURN_SPEED = 600;
const int FAST_RUN_SPEED_MAX = 2500;

const float FAST_RUN_ACCELERATION = 3000;

const int OMEGA_SPIN_TURN = 360;
const int ALPHA_SPIN_TURN = 3600;

const int LINE_FOLLOWER_SPEED_PER_SWITCH =      100; // switch * this = mm/s nominal speed
const int FAST_LINE_FOLLOWER_SPEED_PER_SWITCH = 150; // switch * this = mm/s nominal speed

//***** SENSOR SCALING ******************************************************//
// This is the normalised value seen by the front sensor when the mouse is
// in its calibration position
const int SIDE_NOMINAL = 100;
const int FRONT_NOMINAL = 100;

// The side sensors are not reliable close to a wall ahead. This value
// is the limit for the front sensor reading, above which the side sensors
// are likely to give innaccurate readings because of reflections from
// the wall ahead
const int FRONT_WALL_RELIABILITY_LIMIT = 10000;

// Sensor brightness adjustment factor. The compiler calculates these so it saves processor time
const float FRONT_LEFT_SCALE = (float)FRONT_NOMINAL / FRONT_LEFT_CALIBRATION;
const float FRONT_RIGHT_SCALE = (float)FRONT_NOMINAL / FRONT_RIGHT_CALIBRATION;
const float LEFT_SCALE = (float)SIDE_NOMINAL / LEFT_CALIBRATION;
const float RIGHT_SCALE = (float)SIDE_NOMINAL / RIGHT_CALIBRATION;

// the values above which, a wall is seen
const int LEFT_THRESHOLD =  40;  // minimum value to register a wall
const int RIGHT_THRESHOLD = 40;  // minimum value to register a wall
const int FRONT_THRESHOLD = 35;  // minimum value to register a wall

// the distance through the cell at which the corresponding sensor
// will see a falling edge
const int LEFT_EDGE_POS = 20;
const int RIGHT_EDGE_POS = 20;

// the position in the cell where the sensors are sampled.
const float SENSING_POSITION = 170.0;  // HALF_CELL + 40mm, i.e. around 65mm before a subsequent 1/2 size wall

// clang-format off
// These take no storage - the compiler uses the values directly
const TurnParameters turn_params[4] = {
    //           speed, entry,   exit, angle, omega,  alpha, sensor threshold
    {SEARCH_TURN_SPEED,    70,     70,  90.0, 287.0, 2866.0, TURN_THRESHOLD_SS90E}, // 0 => SS90EL
    {SEARCH_TURN_SPEED,    70,     70, -90.0, 287.0, 2866.0, TURN_THRESHOLD_SS90E}, // 0 => SS90ER
    {SEARCH_TURN_SPEED,    70,     70,  90.0, 287.0, 2866.0, TURN_THRESHOLD_SS90E}, // 0 => SS90L
    {SEARCH_TURN_SPEED,    70,     70, -90.0, 287.0, 2866.0, TURN_THRESHOLD_SS90E}, // 0 => SS90R
};
// clang-format on

//***************************************************************************//
// Battery resistor bridge //Derek Hall//
// The battery measurement is performed by first reducing the battery voltage
// with a potential divider formed by two resistors. Here they are named R1 and R2
// though that may not be their designation on the schematics.
//
// Resistor R1 is the high-side resistor and connects to the battery supply
// Resistor R2 is the low-side resistor and connects to ground
// Battery voltage is measured at the junction of these resistors
// The ADC port used for the conversion will have a full scale reading (FSR) that
// depends on the device being used. Typically that will be 1023 for a 10-bit ADC as
// found on an Arduino but it may be 4095 if you have a 12-bit ADC.
// Finally, the ADC converter on your processor will have a reference voltage. On
// the Arduinos for example, this is 5 Volts. Thus, a full scale reading of
// 1023 would represent 5 Volts, 511 would be 2.5Volts and so on.
//
// in this section you can enter the appropriate values for your ADC and potential
// divider setup to ensure that the battery voltage reading performed by the sensors
// is as accurate as possible.
//
// By calculating the battery multiplier here, you can be sure that the actual
// battery voltage calulation is done as efficiently as possible.
// The compiler will do all these calculations so your program does not have to.

const float BATTERY_R1 = 10000.0;  // resistor to battery +
const float BATTERY_R2 = 10000.0;  // resistor to Gnd
const float BATTERY_DIVIDER_RATIO = BATTERY_R2 / (BATTERY_R1 + BATTERY_R2);
const float ADC_FSR = 1023.0;     // The maximum reading for the ADC
const float ADC_REF_VOLTS = 5.0;  // Reference voltage of ADC

const float BATTERY_MULTIPLIER = (ADC_REF_VOLTS / ADC_FSR / BATTERY_DIVIDER_RATIO);

const int MOTOR_MAX_PWM = 255;

