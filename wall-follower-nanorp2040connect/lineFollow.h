#pragma once
#include "ukmarsbot-pins.h"
#include "quadrature.h"

// Distances during line following
const int ACCELERATION_DISTANCE = 20;
//const int DECELERATION_DISTANCE = 200; //150;
const float DECELERATION_FACTOR = 0.5;      // Distance allowed to ramp down the speed approaching a corner, in mm * speed
const float CORNER_DECL_COAST_FACTOR = 3.0; // Distance after deceleratoin to coast to stabilise speed, in mm * speed
const int STOP_DISTANCE = 80 - CROSSOVER_TOLERANCE;

// PID values
const float PID_Kp = 0.025; //0.012; //0.03; //0.012; // 0.015;    // 0.03 old 1/2 size board
const float PID_Ki = 0.0;
const float PID_Kd = 0.002; //0.001; //0.002; // 0.001;  // 0.0025 old 1/2 size board
const float FEED_FORWARD = 0.0; // speed increase fraction

extern void lineFollower(int basespeed);

// Encoders
extern Quadrature_encoder<m1encoder2, m1encoder1> encoder_l;
extern Quadrature_encoder<m2encoder2, m2encoder1> encoder_r;
