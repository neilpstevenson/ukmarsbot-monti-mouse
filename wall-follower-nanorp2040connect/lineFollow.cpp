#include "Arduino.h"
#include "defaults.h"
#include "lineFollow.h"
#include "motors.h"
#include "pid.h"
#include "pathRecorder.h"

unsigned long int endTime = 0;
unsigned long int startTime = 0;

PathRecorder pathRecorder;

// Initial line follower function, recording what it sees along the way
//
void followAndRecordPath(int basespeed, int slowdownSpeed) 
{
  Serial.print("Base speed: "); Serial.println(basespeed);
  Serial.print("Slowdown speed: "); Serial.println(slowdownSpeed);
  
  // Set up steering PID
  float pidInput = 0.0;
  float pidSetpoint = 0.0;
  PID steeringPID(PID_Kp, PID_Ki, PID_Kd, &pidInput, &pidSetpoint);

  // Reset everthing
  encoder_l.reset_count();
  encoder_r.reset_count();
  pathRecorder.reset();

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

    // Detect and record any markers found 
    pathRecorder.record(lsidesens, rsidesens, (int)(-encoder_l.count() * ENCODER_CALIBRATION), (int)(encoder_r.count() * ENCODER_CALIBRATION));

    // Set the motors to the default speed +/- turn
    rightspeed = int(basespeed * (1 + turn));
    leftspeed = int(basespeed * (1 - turn));
  
    analogWrite(rmotorPWM, rightspeed >= 0 ? std::min(255,rightspeed) : 0); // set right motor speed
    analogWrite(lmotorPWM, leftspeed >= 0 ? std::min(255,leftspeed) : 0); // set left motor speed

    // Put LED into state according to how many markers seen
    digitalWrite(indicatorLedBlue, pathRecorder.currentSegmentNumber() % 2 ? LOW : HIGH); // toggle LED
  
    count++;
    delay(3);
  }

  endTime = millis();

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

    // Set the motors to the default speed +/- turn
    rightspeed = int(slowdownSpeed * (1 + turn));
    leftspeed = int(slowdownSpeed * (1 - turn));
    analogWrite(rmotorPWM, rightspeed >= 0 ? std::min(255,rightspeed) : 0); // set right motor speed
    analogWrite(lmotorPWM, leftspeed >= 0 ? std::min(255,leftspeed) : 0); // set left motor speed
    
    delay(3);
  }
  
  digitalWrite(indicatorLedBlue, LOW); // LED off
  
  stopmotors();

  // Print results
  Serial.print("Time mS: "); Serial.println(endTime-startTime);
  Serial.print("Loop: "); Serial.print(count);
  Serial.print(" ("); Serial.print(count/((endTime-startTime)/1000.0)); Serial.println("/sec)"); 
  pathRecorder.printPath();
}

// Line follow, using path knowledge previously recorded
//
void replayRecordedPath(int forwardSpeed, int cornerApproachSpeed, int cornerSpeed, int slowdownSpeed)
{
  Serial.print("Time mS: "); Serial.println(endTime-startTime);
  pathRecorder.printPath();
  delay(1000);

  Serial.print("Base speed: "); Serial.println(forwardSpeed);
  Serial.print("Corner speed: "); Serial.println(cornerSpeed);
  Serial.print("Slowdown speed: "); Serial.println(slowdownSpeed);

  // Temporary record while playing back
  PathRecorder playbackRecorder;

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

    float turn = steeringPID.compute();

    // Set the motors to the default speed +/- turn
    if(PathRecorder::isDirectionForward(currentDirection))
    {
      long decelCoastDistance = (forwardSpeed - cornerApproachSpeed) * CORNER_DECL_COAST_FACTOR;
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
        Serial.print("Decel,");
      }
      else if(distanceToGo > decelDistance)
      {
        if(playbackRecorder.getCurrentSegmentDistance() < ACCELERATION_DISTANCE)
        {
          // Accelerate up to speed
          int speed = (forwardSpeed - cornerSpeed) * playbackRecorder.getCurrentSegmentDistance() / ACCELERATION_DISTANCE + cornerSpeed;
          // Keep going if already up to speed
          currentSpeed = std::max(speed, currentSpeed);
          Serial.print("Accel,");
        }
        else
        {
          // Want max speed
          currentSpeed = forwardSpeed;
          Serial.print("Fast,");
        }
      }
      else
      {
        currentSpeed = cornerSpeed;
        Serial.print("D_done,");
      }
    }
    else
    {
      currentSpeed = cornerSpeed;
      Serial.print("Corner,");

      // Add a bit of feed-forward
      if(currentDirection == PathRecorder::rightTurn)
        turn -= FEED_FORWARD;
      else if(currentDirection == PathRecorder::leftTurn)
        turn += FEED_FORWARD;
    }
    
    rightspeed = int(currentSpeed * (1 + turn));
    leftspeed = int(currentSpeed * (1 - turn));
 
    analogWrite(rmotorPWM, rightspeed >= 0 ? std::min(255,rightspeed) : 0); // set right motor speed
    analogWrite(lmotorPWM, leftspeed >= 0 ? std::min(255,leftspeed) : 0); // set left motor speed

    // Put LED into state according to how many markers seen
    digitalWrite(indicatorLedBlue, playbackRecorder.currentSegmentNumber() % 2 ? LOW : HIGH); // toggle LED

    // Debug
    Serial.print(playbackRecorder.currentSegmentNumber());
    Serial.print(",");        PathRecorder::printDirection(currentDirection);
    Serial.print(",seg_d=");  Serial.print(pathRecorder.getSegmentDistance());
    Serial.print(",cur_d=");  Serial.print(playbackRecorder.getCurrentSegmentDistance());
    Serial.print(",s=");      Serial.print(currentSpeed);
    Serial.print(",turn=");   Serial.print(turn);
    Serial.print(",pos=");    Serial.print(leftPosition);
    Serial.print(",");        Serial.println(leftPosition);

    count++;
    delay(3);
  }
  
  endTime = millis();

  Serial.print("Slowdown ");
  Serial.println(slowdownSpeed);

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

    // Set the motors to the default speed +/- turn
    rightspeed = slowdownSpeed * (1 + turn);
    leftspeed = slowdownSpeed * (1 - turn);
    analogWrite(rmotorPWM, rightspeed >= 0 ? std::min(255,rightspeed) : 0); // set right motor speed
    analogWrite(lmotorPWM, leftspeed >= 0 ? std::min(255,leftspeed) : 0); // set left motor speed
    
    delay(3);
  }
 
  digitalWrite(indicatorLedBlue, LOW); // LED off
  
  stopmotors();

  // Print results
  Serial.print("Time mS: "); Serial.println(endTime-startTime);
  Serial.print("Loop: "); Serial.print(count);
  Serial.print(" ("); Serial.print(count/((endTime-startTime)/1000.0)); Serial.println("/sec)"); 
  pathRecorder.printPath();
}


void lineFollower(int basespeed)
{
    // Initial run
    followAndRecordPath(basespeed, (int)(basespeed*SLOWDOWN_SPEED_RATIO));
  
    while(1)
    {
      // Now replay at higher speed
      buttonwait(20); // wait for function button to be pressed
      batterycheck();
      functionswitch(); // read function switch value after button released
      int fastRunSpeed = fnswvalue * 17;
      if(fastRunSpeed <= basespeed)
        fastRunSpeed = basespeed + 17;
      //delay(2000);
      replayRecordedPath(fastRunSpeed, basespeed - 16, basespeed, (int)(basespeed*SLOWDOWN_SPEED_RATIO));
    }
}