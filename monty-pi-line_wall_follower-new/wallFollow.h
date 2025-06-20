#pragma once

//#include "ukmarsbot-pins.h"
//#include "quadrature.h"
#include "wallfollow-config.h"

// Follow the left wall, return TRUE if successful
extern bool FollowLeftWall();
extern void simpleWallFollower(int basespeed);

/*
// Encoders
extern Quadrature_encoder<m1encoder2, m1encoder1> encoder_l;
extern Quadrature_encoder<m2encoder2, m2encoder1> encoder_r;

// Sensors
inline int sensorLeft()
{
  return int(lfrontsens * sensor_calibrate_l);
}

inline int sensorRight()
{
  return int(rsidesens * sensor_calibrate_r);
}

inline int sensorForward()
{
  return int(rfrontsens * sensor_calibrate_f);
}

*/