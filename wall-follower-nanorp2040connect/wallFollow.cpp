#include "Arduino.h"
#include "defaults.h"
#include "wallFollow.h"
#include "motors.h"
#include "pid.h"

void logSensors(const char *mode)
{
  static bool firstLog = true;
  if(firstLog)
  {
    firstLog = false;
    // Print header
    DebugPort.print("Mode,LeftCal,FrontCal,RightCal,EncLCal,EncRCal");
#ifdef LOG_RAW_SENSORS  
    DebugPort.println(",LeftRaw,FrontRaw,RightRaw,EncLRaw,EncRRaw");
#else
    DebugPort.println();
#endif
  }

  DebugPort.print(mode);            DebugPort.print(",");
  
  // Sensors
  DebugPort.print(sensorLeft());     DebugPort.print(",");
  DebugPort.print(sensorForward());  DebugPort.print(",");
  DebugPort.print(sensorRight());    DebugPort.print(",");
  
  // Encoders
  DebugPort.print(encoder_l.count() * encode_calibrate_l);  DebugPort.print("mm,");
  DebugPort.print(encoder_r.count() * encode_calibrate_r);  DebugPort.print("mm,");

#ifdef LOG_RAW_SENSORS  
  DebugPort.print(sensors.left().getRaw());       DebugPort.print(",");
  DebugPort.print(sensors.frontLeft().getRaw());  DebugPort.print(",");
  DebugPort.print(sensors.frontRight().getRaw()); DebugPort.print(",");
  DebugPort.print(sensors.right().getRaw());      DebugPort.print(",");
  
  // Encoders
  DebugPort.print(encoder_l.count());             DebugPort.print(",");
  DebugPort.print(encoder_r.count());
#endif

  DebugPort.println();
}

// Simple wall-follower function
//
void simpleWallFollower(int basespeed) 
{
  int sensdiff = 0;
  int leftTurnCount = 0;
  int leftTurnPosition = 0;

  motors.begin();

  DebugPort.print("Wall Follower, speed: "); DebugPort.println(basespeed);
  
  // Set up steering PID
  float pidInput = 0.0;
  float pidSetpoint = 0.0;
  PID steeringPID(wall_follow_kp, 0, wall_follow_kd, &pidInput, &pidSetpoint);

  // Reset everthing
  encoder_l.reset_count();
  encoder_r.reset_count();
 
  // Indicators
  digitalWrite (indicatorLedBlue, LOW);

  // Set up motor direction
//  digitalWrite(rmotorDIR, HIGH); // set right motor forward
//  digitalWrite(lmotorDIR, LOW); // set left motor forward

  // Forward to start line
  rightspeed = basespeed;
  leftspeed = basespeed;
  motors.forwardPower(basespeed);
  //analogWrite(rmotorPWM, rightspeed); // set right motor speed
  //analogWrite(lmotorPWM, leftspeed); // set left motor speed

  while(true)
  {
    photoread(true);
    sensdiff = lfrontsens - wallFollowerTargetDistance;
    bool forwardBlocked = rfrontsens > wallFollowerForwardAvoidDistance;
    bool leftGap = lfrontsens < wallFollowerLeftGapThreshold;

    static float sensdiffFitered;
    sensdiffFitered = sensdiffFitered * (1.0 - wallFollowerSensorFilter) + sensdiff * wallFollowerSensorFilter;

    //DebugPort.println(lfrontsens);
    //DebugPort.println(sensdiff);

    // Push through PID controller
    pidInput = sensdiffFitered;
    float turn = steeringPID.compute() * basespeed;

    // Set the motors to the default speed +/- turn
    if(!leftGap)
    {
      if(!forwardBlocked)
      {
        // Forward
        //
        // Limit the turn to +/-35%
        turn = std::max(std::min(turn, wallFollowerMaxPidTurn), -wallFollowerMaxPidTurn);
        // Keep on following left wall
        rightspeed = int(basespeed * (1 + turn));
        leftspeed = int(basespeed * (1 - turn));

        // We've seen a wall, reset the coast counter
        leftTurnCount = 0;
        leftTurnPosition = encoder_l.count();;

        digitalWrite (sensorLED1, LOW);  // Right/Red LED
        digitalWrite (sensorLED2, LOW);   // Left/Green LED
        digitalWrite (indicatorLedBlue, LOW);  // Centre/Blue LED
      }
      else
      {
        // Blocked ahead - turn right
        rightspeed = -int(basespeed * 1.4);// 0.8);
        leftspeed = int(basespeed * 0.4); //0.8);

        // May need a very short turn
        leftTurnCount = wallFollowerLeftTurnDelay;
        leftTurnPosition = encoder_l.count();

        digitalWrite (sensorLED1, HIGH);  // Right/Red LED
        digitalWrite (sensorLED2, LOW);   // Left/Green LED
        digitalWrite (indicatorLedBlue, LOW);  // Centre/Blue LED
      }
    }
//    else if(++leftTurnCount <= wallFollowerLeftTurnDelay)
    else if(leftTurnPosition - wallFollowerLeftTurnDelay /*wallFollowerLeftTurnDelay*/ <= encoder_l.count())
    {
      // Gap on left, but keep going ahead a small amount first
      // slightly right
      rightspeed = int(basespeed);// * 0.95);
      leftspeed = int(basespeed);

      digitalWrite (sensorLED1, LOW);  // Right/Red LED
      digitalWrite (sensorLED2, HIGH);   // Left/Green LED
      digitalWrite (indicatorLedBlue, HIGH);  // Centre/Blue LED
    }
    else
    {
      // Gap on left, turn into it now

      // We want a constant velocity, so
      // Vright = V + V/R.d/2   where V is forward velocity, R the requred turning radius and d the mouse effective diameter
      // and
      // Vleft = V - V/R.d/2
      // If R is 90 and r is 40 then 
      // Vleft = 0.555V and Vright = 1.444V
      float vr = 1 + 1/(90.0 - basespeed * wallFollowerLeftTurnInertiaCompensation) * turning_diameter_mm/2;
      float vl = 1 - 1/(90.0 - basespeed * wallFollowerLeftTurnInertiaCompensation) * turning_diameter_mm/2;
      rightspeed = int(basespeed * vr);
      leftspeed = int(basespeed * vl);

      digitalWrite (sensorLED1, LOW);  // Right/Red LED
      digitalWrite (sensorLED2, HIGH);   // Left/Green LED
      digitalWrite (indicatorLedBlue, LOW);  // Centre/Blue LED
    }

    // Update motors
    motors.right.setPower(rightspeed);
    motors.left.setPower(-leftspeed);
/*    
    if(rightspeed >= 0)
    {
      digitalWrite(rmotorDIR, HIGH); // set right motor forward
      analogWrite(rmotorPWM, rightspeed); // set right motor speed
    }
    else
    {
      digitalWrite(rmotorDIR, LOW); // set right motor reverse
      analogWrite(rmotorPWM, -rightspeed); // set right motor speed
    }

    if(leftspeed >= 0)
    {
      digitalWrite(lmotorDIR, LOW); // set left motor forward
      analogWrite(lmotorPWM, leftspeed); // set left motor speed
    }
    else
    {
      digitalWrite(lmotorDIR, HIGH); // set left motor reverse
      analogWrite(lmotorPWM, -leftspeed); // set left motor speed
    }
*/
    delay(3);
  }
}

