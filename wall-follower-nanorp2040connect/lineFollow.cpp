#include "Arduino.h"
#include "defaults.h"
#include "lineFollow.h"
#include "motors.h"
#include "pid.h"
#include "pathRecorder.h"

unsigned long int endTime = 0;
unsigned long int startTime = 0;
int markerLowThreshold = defaultMarkerLowThreshold;

// Log sensor state and current speeds
void logFollowerState(int segment, float turn)
{
  static bool logHeading = true;
  static const int logFreq = 0; //5;  // Number of loops between logs
  static int count = 0;
  if(logHeading)
  { 
    logHeading = false;
    DebugPort.println("seg,lf,rf,turn,lsp,rsp");
  }

  if(++count == logFreq)
  {
    count = 0;
    DebugPort.print(segment);           DebugPort.print(","); 
    DebugPort.print(lfrontsens);        DebugPort.print(","); 
    DebugPort.print(rfrontsens);        DebugPort.print(","); 
    DebugPort.print(turn);              DebugPort.print(","); 
    DebugPort.print(encoder_l.speed()); DebugPort.print(","); 
    DebugPort.print(encoder_r.speed()); DebugPort.print(","); 
    DebugPort.println();
  }
}

// Initial line follower function, recording what it sees along the way
//
void followAndRecordPath(PathRecorder &pathRecorder, int basespeed, int slowdownSpeed, bool pursuitMode) 
{
  DebugPort.print("Base speed: "); DebugPort.println(basespeed);
  DebugPort.print("Slowdown speed: "); DebugPort.println(slowdownSpeed);
  
  // Set up steering PID
  float pidInput = 0.0;
  float pidSetpoint = 0.0;
  PID steeringPID(PID_Kp, PID_Ki, PID_Kd, &pidInput, &pidSetpoint);

  // Reset everthing
  encoder_l.reset_count();
  encoder_r.reset_count();
  pathRecorder.reset(pursuitMode);

  // Set up motor direction
  digitalWrite(rmotorDIR, HIGH); // set right motor forward
  digitalWrite(lmotorDIR, LOW); // set left motor forward

  startTime = millis();
  unsigned long int count = 0;

  // Forward to start line
  rightspeed = basespeed;
  leftspeed = basespeed;
  analogWrite(rmotorPWM, rightspeed); // set right motor speed
  analogWrite(lmotorPWM, leftspeed); // set left motor speed

  while(!pathRecorder.detectedEndMarker())
  {
    photoread();
#if SENSOR_POLAIRTY_TRUE
    sensdiff = rfrontsens - lfrontsens;
#else
    sensdiff = lfrontsens - rfrontsens;
#endif

    // Push through PID controller
    pidInput = sensdiff;
    float turn = steeringPID.compute();

    // Adjust the turn PID output based on new speed
    turn = constrain(turn * (1 + (basespeed - 64) * PID_TURN_FACTOR), -MAX_MOTOR_VOLTS, MAX_MOTOR_VOLTS);

    // Detect and record any markers found 
    pathRecorder.record(lsidesens, rsidesens, (int)(-encoder_l.count() * ENCODER_CALIBRATION), (int)(encoder_r.count() * ENCODER_CALIBRATION));

    // Set the motors to the default speed +/- turn
    rightspeed = int(basespeed * (1 + turn));
    leftspeed = int(basespeed * (1 - turn));
  
    analogWrite(rmotorPWM, rightspeed >= 0 ? std::min(255,rightspeed) : 0); // set right motor speed
    analogWrite(lmotorPWM, leftspeed >= 0 ? std::min(255,leftspeed) : 0); // set left motor speed

    // Put LED into state according to how many markers seen
    digitalWrite(indicatorLedBlue, pathRecorder.currentSegmentNumber() % 2 ? LOW : HIGH); // toggle LED
  
    logFollowerState(pathRecorder.currentSegmentNumber(), turn);

    count++;
    delay(3);
  }

  endTime = millis();

  if(!pursuitMode)
  {
    // Slowdown sequence
    int stopMarkerPos_r = encoder_r.count();
    while(stopMarkerPos_r + STOP_DISTANCE > encoder_r.count())
    {
      photoread();

  #if SENSOR_POLAIRTY_TRUE
      sensdiff = rfrontsens - lfrontsens;
  #else
      sensdiff = lfrontsens - rfrontsens;
  #endif

      // Push through PID controller
  #ifdef DETECT_CROSSOVER_TWITCH    
  #if SENSOR_POLAIRTY_TRUE
      if(rsidesens < markerHighThreshold ||
        lsidesens < markerHighThreshold)
  #else
      if(rsidesens > markerLowThreshold || 
        lsidesens > markerLowThreshold)
  #endif
      {
        // Not at crossover
        pidInput = sensdiff;
      }
      else
      {
        // Ignore crossover
        pidInput = 0;
      }
  #else
      pidInput = sensdiff;
  #endif    
      float turn = steeringPID.compute();

    // Adjust the turn PID output based on new speed
      turn = constrain(turn * (1 + (slowdownSpeed - 64) * PID_TURN_FACTOR), -MAX_MOTOR_VOLTS, MAX_MOTOR_VOLTS);

      // Set the motors to the default speed +/- turn
      rightspeed = int(slowdownSpeed * (1 + turn));
      leftspeed = int(slowdownSpeed * (1 - turn));
      analogWrite(rmotorPWM, rightspeed >= 0 ? std::min(255,rightspeed) : 0); // set right motor speed
      analogWrite(lmotorPWM, leftspeed >= 0 ? std::min(255,leftspeed) : 0); // set left motor speed

      logFollowerState(pathRecorder.currentSegmentNumber(), turn);
      
      delay(3);
    }
    
    digitalWrite(indicatorLedBlue, LOW); // LED off
    
    stopmotors();
  }
  else
  {
    // Persuit mode
    DebugPort.print("Pursuit mode ended");
    digitalWrite(indicatorLedBlue, LOW); // LED off
  }

  // Print results
  DebugPort.print("Time mS: "); DebugPort.println(endTime-startTime);
  DebugPort.print("Loop: "); DebugPort.print(count);
  DebugPort.print(" ("); DebugPort.print(count/((endTime-startTime)/1000.0)); DebugPort.println("/sec)"); 
  pathRecorder.printPath();
}

// Line follow, using path knowledge previously recorded
//
void replayRecordedPath(PathRecorder &pathRecorder, int forwardSpeed, int cornerApproachSpeed, int cornerSpeed, int slowdownSpeed, bool pursuitMode)
{
  DebugPort.print("Time mS: "); DebugPort.println(endTime-startTime);
  if(!pursuitMode)
  {
    pathRecorder.printPath();
    delay(1000);
  }

  DebugPort.print("Base speed: "); DebugPort.println(forwardSpeed);
  DebugPort.print("Corner speed: "); DebugPort.println(cornerSpeed);
  DebugPort.print("Slowdown speed: "); DebugPort.println(slowdownSpeed);

  // Temporary record while playing back
  PathRecorder playbackRecorder;
  playbackRecorder.reset(pursuitMode);

  // Set up steering PID
  float pidInput = 0.0;
  float pidSetpoint = 0.0;
  PID steeringPID(PID_Kp, PID_Ki, PID_Kd, &pidInput, &pidSetpoint);

  // Reset everthing
  encoder_l.reset_count();
  encoder_r.reset_count();

  // Set up motor direction
  digitalWrite(rmotorDIR, HIGH); // set right motor forward
  digitalWrite(lmotorDIR, LOW); // set left motor forward

  startTime = millis();
  unsigned long int count = 0;

  // Start at cornering speed
  int currentSpeed = cornerSpeed;

  // Forward to start line
  rightspeed = currentSpeed;
  leftspeed = currentSpeed;
  analogWrite(rmotorPWM, rightspeed >= 0 ? std::min(255,rightspeed) : 0); // set right motor speed
  analogWrite(lmotorPWM, leftspeed >= 0 ? std::min(255,leftspeed) : 0); // set left motor speed

  PathRecorder::SegmentDirection currentDirection = pathRecorder.getFirstSegment();
  PathRecorder::SegmentDirection nextDirection = pathRecorder.peakNextSegment();
  
  while(!playbackRecorder.detectedEndMarker())
  {
    photoread();
#if SENSOR_POLAIRTY_TRUE
    sensdiff = rfrontsens - lfrontsens;
#else
    sensdiff = lfrontsens - rfrontsens;
#endif

    // Detect and record any markers found 
    int leftPosition = (int)(-encoder_l.count() * ENCODER_CALIBRATION);
    int rightPosition = (int)(encoder_r.count() * ENCODER_CALIBRATION);
    playbackRecorder.record(lsidesens, rsidesens, leftPosition, rightPosition);

    // Adjust the segment number, if changed
    if(currentDirection != PathRecorder::SegmentDirection::endMark && pathRecorder.currentSegmentNumber() != playbackRecorder.currentSegmentNumber())
    {
      currentDirection = pathRecorder.getNextSegment();
      nextDirection = pathRecorder.peakNextSegment();
    }

    // Push through PID controller
#ifdef DETECT_CROSSOVER_TWITCH    
#if SENSOR_POLAIRTY_TRUE
    if(rsidesens < markerHighThreshold ||
       lsidesens < markerHighThreshold)
#else
    if(rsidesens > markerLowThreshold || 
       lsidesens > markerLowThreshold)
#endif
    {
      // Not at crossover
      pidInput = sensdiff;
    }
    else
    {
      // Ignore crossover
      pidInput = 0;
    }
#else
    pidInput = sensdiff;
#endif    

    float turn = steeringPID.compute();

    // Set the motors to the default speed +/- turn
    if(PathRecorder::isDirectionForward(currentDirection))
    {
      long decelCoastDistance = (forwardSpeed - cornerApproachSpeed) * CORNER_DECL_COAST_FACTOR + CORNER_DECL_COAST_DISTANCE;
      long decelDistance = (forwardSpeed - cornerApproachSpeed) * DECELERATION_FACTOR;
      long distanceToGo = pathRecorder.getSegmentDistance() - playbackRecorder.getCurrentSegmentDistance() - decelCoastDistance;
      if(PathRecorder::isDirectionForward(nextDirection))
      {
        distanceToGo += pathRecorder.getNextSegmentDistance();
      }
      if(distanceToGo > 0 && distanceToGo <= decelDistance) 
      {
        // Decelerate
        int speed = (forwardSpeed - cornerApproachSpeed) * distanceToGo / decelDistance + cornerApproachSpeed;
        // Don't go faster if already slower
        currentSpeed = std::min(speed, currentSpeed);
        //DebugPort.print("Decel,");
      }
      else if(distanceToGo > decelDistance)
      {
        if(playbackRecorder.getCurrentSegmentDistance() < ACCELERATION_DISTANCE)
        {
          // Accelerate up to speed
          int speed = (forwardSpeed - cornerSpeed) * playbackRecorder.getCurrentSegmentDistance() / ACCELERATION_DISTANCE + cornerSpeed;
          // Keep going if already up to speed
          currentSpeed = std::max(speed, currentSpeed);
          //DebugPort.print("Accel,");
        }
        else
        {
          // Want max speed
          currentSpeed = forwardSpeed;
          //DebugPort.print("Fast,");
        }
      }
      else
      {
        currentSpeed = cornerSpeed;
        //DebugPort.print("D_done,");
      }
    }
    else
    {
      currentSpeed = cornerSpeed;
      //DebugPort.print("Corner,");

      // Add a bit of feed-forward
      if(currentDirection == PathRecorder::rightTurn)
        turn -= FEED_FORWARD;
      else if(currentDirection == PathRecorder::leftTurn)
        turn += FEED_FORWARD;
    }

    // Adjust the turn PID output based on new speed
    turn = constrain(turn * (1 + (currentSpeed - 64) * PID_TURN_FACTOR), -MAX_MOTOR_VOLTS, MAX_MOTOR_VOLTS);

    rightspeed = int(currentSpeed * (1 + turn));
    leftspeed = int(currentSpeed * (1 - turn));
 
    analogWrite(rmotorPWM, rightspeed >= 0 ? std::min(255,rightspeed) : 0); // set right motor speed
    analogWrite(lmotorPWM, leftspeed >= 0 ? std::min(255,leftspeed) : 0); // set left motor speed

    // Put LED into state according to how many markers seen
    digitalWrite(indicatorLedBlue, playbackRecorder.currentSegmentNumber() % 2 ? LOW : HIGH); // toggle LED

    logFollowerState(playbackRecorder.currentSegmentNumber(), turn);
    
/*
    // Debug
    DebugPort.print(playbackRecorder.currentSegmentNumber());
    DebugPort.print(",");        PathRecorder::printDirection(currentDirection);
    DebugPort.print(",seg_d=");  DebugPort.print(pathRecorder.getSegmentDistance());
    DebugPort.print(",cur_d=");  DebugPort.print(playbackRecorder.getCurrentSegmentDistance());
    DebugPort.print(",s=");      DebugPort.print(currentSpeed);
    DebugPort.print(",turn=");   DebugPort.print(turn);
    DebugPort.print(",pos=");    DebugPort.print(leftPosition);
    DebugPort.print(",");        DebugPort.println(leftPosition);
*/

    count++;
    delay(3);
  }
  
  endTime = millis();

  if(!pursuitMode)
  {
    DebugPort.print("Slowdown ");
    DebugPort.println(slowdownSpeed);

    // Slowdown sequence
    int stopMarkerPos_r = encoder_r.count();
    while(stopMarkerPos_r + STOP_DISTANCE > encoder_r.count())
    {
      photoread();

  #if SENSOR_POLAIRTY_TRUE
      sensdiff = rfrontsens - lfrontsens;
  #else
      sensdiff = lfrontsens - rfrontsens;
  #endif

      // Push through PID controller
      pidInput = sensdiff;
      float turn = steeringPID.compute();

      // Adjust the turn PID output based on new speed
      turn = constrain(turn * (1 + (slowdownSpeed - 64) * PID_TURN_FACTOR), -MAX_MOTOR_VOLTS, MAX_MOTOR_VOLTS);

      // Set the motors to the default speed +/- turn
      rightspeed = slowdownSpeed * (1 + turn);
      leftspeed = slowdownSpeed * (1 - turn);
      analogWrite(rmotorPWM, rightspeed >= 0 ? std::min(255,rightspeed) : 0); // set right motor speed
      analogWrite(lmotorPWM, leftspeed >= 0 ? std::min(255,leftspeed) : 0); // set left motor speed
      
      logFollowerState(playbackRecorder.currentSegmentNumber(), turn);
      
      delay(3);
    }
  
    digitalWrite(indicatorLedBlue, LOW); // LED off
    
    stopmotors();
  }
  else
  {
    // Persuit mode
    DebugPort.print("Pursuit mode ended");
    digitalWrite(indicatorLedBlue, LOW); // LED off
  }

  // Print results
  DebugPort.print("Time mS: "); DebugPort.println(endTime-startTime);
  DebugPort.print("Loop: "); DebugPort.print(count);
  DebugPort.print(" ("); DebugPort.print(count/((endTime-startTime)/1000.0)); DebugPort.println("/sec)"); 
  pathRecorder.printPath();
}


void lineFollower(int basespeed, bool pursuitMode)
{
    // Initial run
    PathRecorder pathRecorder;
    followAndRecordPath(pathRecorder, basespeed, (int)(basespeed*SLOWDOWN_SPEED_RATIO), pursuitMode);
  
    while(1)
    {
      // Now replay at higher speed
      if(!pursuitMode)
      {
        buttonwait(20); // wait for function button to be pressed
      }
      batterycheck();
      functionswitch(); // read function switch value after button released
      int fastRunSpeed = fnswvalue * 17;
      if(fastRunSpeed <= basespeed)
        fastRunSpeed = basespeed + 17;
      //delay(2000);

      replayRecordedPath(pathRecorder, fastRunSpeed, basespeed - CORNER_APPROACH_SLOWDOWN, basespeed, (int)(basespeed*SLOWDOWN_SPEED_RATIO), pursuitMode);
    }
}