#pragma once
//#include "ukmarsbot-pins.h"
//#include "quadrature.h"
//#include "pid.h"

// Distances during line following
const int ACCELERATION_DISTANCE = 20;
const int CORNER_APPROACH_SLOWDOWN = 0; //16; // Amount to slow down after a high speed section before hittind a corner
//const int DECELERATION_DISTANCE = 200; //150;
const float DECELERATION_FACTOR = 0.15;      // Distance allowed to ramp down the speed approaching a corner, in mm * speed
const float CORNER_DECL_COAST_FACTOR = 0.0; //4.0; // Distance after deceleration to coast to stabilise speed, in mm * speed
const int CORNER_DECL_COAST_DISTANCE = 300; // Distance after deceleration to coast in mm
const int STOP_DISTANCE = 100 - CROSSOVER_TOLERANCE;
const float PID_TURN_FACTOR = 0.005;  // Amount to multiply the PID turn result by speed difference of 64 - approx doubling of values from 64 to 255

// PID values
//const float PID_Kp = 0.0025; //0.025; //0.0025; //0.012; //0.03; //0.012; // 0.015;    // 0.03 old 1/2 size board
//const float PID_Ki = 0.0;
//const float PID_Kd = 0.0001; //0.002; //0.0001; //0.001; //0.002; // 0.001;  // 0.0025 old 1/2 size board
//const float FEED_FORWARD = 0.0; // speed increase fraction

//extern void lineFollower(int basespeed, bool pursuitMode);

unsigned long int endTime = 0;
unsigned long int startTime = 0;
int markerLowThreshold = defaultMarkerLowThreshold;

// Log sensor state and current speeds
void logFollowerState(int segment, float turn)
{
  /* TODO
  static bool logHeading = true;
  static const int logFreq = 0; //5;  // Number of loops between logs
  static int count = 0;
  if(logHeading)
  { 
    logHeading = false;
    SerialPort.println("seg,lf,rf,turn,lsp,rsp");
  }

  if(++count == logFreq)
  {
    count = 0;
    SerialPort.print(segment);           SerialPort.print(","); 
    SerialPort.print(lfrontsens);        SerialPort.print(","); 
    SerialPort.print(rfrontsens);        SerialPort.print(","); 
    SerialPort.print(turn);              SerialPort.print(","); 
    SerialPort.print(encoder_l.speed()); SerialPort.print(","); 
    SerialPort.print(encoder_r.speed()); SerialPort.print(","); 
    SerialPort.println();
  }
  */
}

// Initial line follower function, recording what it sees along the way
//
void followAndRecordPath(PathRecorder &pathRecorder, int basespeed, int slowdownSpeed, bool pursuitMode) 
{
  SerialPort.print("Base speed: "); SerialPort.println(basespeed);
  SerialPort.print("Slowdown speed: "); SerialPort.println(slowdownSpeed);
  
  // // Set up steering PID
  // float pidInput = 0.0;
  // float pidSetpoint = 0.0;
  // PID steeringPID(PID_Kp, PID_Ki, PID_Kd, &pidInput, &pidSetpoint);

  // Reset everthing
  motion.set_position(0);
  rotation.set_position(0);
  pathRecorder.reset(pursuitMode);
  encoders.reset();

  // Start the controllers
  sensors.enable();
  motion.reset_drive_system();
  sensors.set_steering_mode(STEER_LINE_FOLLOW);

  startTime = millis();
  unsigned long int count = 0;

  // Forward to start line
  rightspeed = basespeed;
  leftspeed = basespeed;
  motion.start_move(20, basespeed, basespeed, SEARCH_ACCELERATION);
  //analogWrite(rmotorPWM, rightspeed); // set right motor speed
  //analogWrite(lmotorPWM, leftspeed); // set left motor speed

  while(!pathRecorder.detectedEndMarker())
  {
    //photoread();
// #if SENSOR_POLAIRTY_TRUE
//     sensdiff = sensors.get_front_diff();
// #else
//     sensdiff = -sensors.get_front_diff();
// #endif

    // // Push through PID controller
    // pidInput = sensdiff;
    // float turn = steeringPID.compute();

    // Adjust the turn PID output based on new speed
//    turn = constrain(turn * (1 + (basespeed - 64) * PID_TURN_FACTOR), -MAX_MOTOR_VOLTS, MAX_MOTOR_VOLTS);

    // Detect and record any markers found 
    pathRecorder.record(sensors.see_left_wall?1000:0, sensors.see_right_wall?1000:0, encoders.robot_distance(), encoders.robot_angle());
    //(int)(-encoder_l.count() * ENCODER_CALIBRATION), (int)(encoder_r.count() * ENCODER_CALIBRATION));

    // Set the motors to the default speed +/- turn
//    SerialPort.print("turn=");    SerialPort.println(turn);
    //motion.turn(turn, basespeed, basespeed, 2866);
    // rightspeed = int(basespeed * (1 + turn));
    // leftspeed = int(basespeed * (1 - turn));
    //motion.start_move(10, basespeed, basespeed, SEARCH_ACCELERATION);
    //analogWrite(rmotorPWM, rightspeed >= 0 ? std::min(255,rightspeed) : 0); // set right motor speed
    //analogWrite(lmotorPWM, leftspeed >= 0 ? std::min(255,leftspeed) : 0); // set left motor speed

    // Put LED into state according to how many markers seen
    digitalWrite(indicatorLedBlue, pathRecorder.currentSegmentNumber() % 2 ? LOW : HIGH); // toggle LED
  
 //   logFollowerState(pathRecorder.currentSegmentNumber(), turn);
    count++;
    delay(2);
  }

  endTime = millis();

  if(!pursuitMode)
  {
    // Slowdown sequence
//    int stopMarkerPos_r = encoder_r.count();
//    while(stopMarkerPos_r + STOP_DISTANCE > encoder_r.count())
    int stopMarkerPos_r = forward.position();
    while(stopMarkerPos_r + STOP_DISTANCE > forward.position())
    {
      //photoread();

  //#if SENSOR_POLAIRTY_TRUE
//      sensdiff = sensors.get_front_diff();
  //#else
    //  sensdiff = -sensors.get_front_diff();
  //#endif

      // Push through PID controller
  // #ifdef DETECT_CROSSOVER_TWITCH    
  // #if SENSOR_POLAIRTY_TRUE
  //     if(rsidesens < markerHighThreshold ||
  //       lsidesens < markerHighThreshold)
  // #else
  //     if(rsidesens > markerLowThreshold || 
  //       lsidesens > markerLowThreshold)
  // #endif
  //     {
  //       // Not at crossover
  //       pidInput = sensdiff;
  //     }
  //     else
  //     {
  //       // Ignore crossover
//        pidInput = 0;
//      }
//  #else
//      pidInput = sensdiff;
//  #endif    
//      float turn = steeringPID.compute();

    // Adjust the turn PID output based on new speed
//      turn = constrain(turn * (1 + (slowdownSpeed - 64) * PID_TURN_FACTOR), -MAX_MOTOR_VOLTS, MAX_MOTOR_VOLTS);

      // Set the motors to the default speed +/- turn
      // rightspeed = int(slowdownSpeed * (1 + turn));
      // leftspeed = int(slowdownSpeed * (1 - turn));
      motion.set_target_velocity(slowdownSpeed);
      //analogWrite(rmotorPWM, rightspeed >= 0 ? std::min(255,rightspeed) : 0); // set right motor speed
      //analogWrite(lmotorPWM, leftspeed >= 0 ? std::min(255,leftspeed) : 0); // set left motor speed

//      logFollowerState(pathRecorder.currentSegmentNumber(), turn);
      
      delay(2);
    }
    
    digitalWrite(indicatorLedBlue, LOW); // LED off
    
    SerialPort.println(F("Line track circuit completed"));
    motion.stop_move();
    delay(100);
    while(!motion.move_finished())
    {
      delay(2);
    }
    sensors.disable();
    motion.reset_drive_system();
    motion.disable_drive();
    sensors.set_steering_mode(STEERING_OFF);
  }
  else
  {
    // Persuit mode
    SerialPort.print("Pursuit mode ended");
    digitalWrite(indicatorLedBlue, LOW); // LED off
  }

  // Print results
  SerialPort.print("Time mS: "); SerialPort.println(endTime-startTime);
  SerialPort.print("Loop: "); SerialPort.print(count);
  SerialPort.print(" ("); SerialPort.print(count/((endTime-startTime)/1000.0)); SerialPort.println("/sec)"); 
  pathRecorder.printPath();
}

// Line follow, using path knowledge previously recorded
//
void replayRecordedPath(PathRecorder &pathRecorder, int forwardSpeed, int cornerApproachSpeed, int cornerSpeed, int slowdownSpeed, bool pursuitMode)
{
  SerialPort.print("Time mS: "); SerialPort.println(endTime-startTime);
  if(!pursuitMode)
  {
    pathRecorder.printPath();
    delay(1000);
  }

  SerialPort.print("Base speed: "); SerialPort.println(forwardSpeed);
  SerialPort.print("Corner speed: "); SerialPort.println(cornerSpeed);
  SerialPort.print("Slowdown speed: "); SerialPort.println(slowdownSpeed);

  // Temporary record while playing back
  PathRecorder playbackRecorder;
  playbackRecorder.reset(pursuitMode);

  // Set up steering PID
  // float pidInput = 0.0;
  // float pidSetpoint = 0.0;
  // PID steeringPID(PID_Kp, PID_Ki, PID_Kd, &pidInput, &pidSetpoint);

  // Reset everthing
  motion.set_position(0);
  encoders.reset();

  // Start the controllers
  sensors.enable();
  if(!pursuitMode)
  {
    motion.reset_drive_system();
  }
  sensors.set_steering_mode(STEER_LINE_FOLLOW);

  startTime = millis();
  unsigned long int count = 0;

  // Start at cornering speed
  int currentSpeed = cornerSpeed;

  // Forward to start line
  rightspeed = currentSpeed;
  leftspeed = currentSpeed;
  motion.start_move(20, cornerSpeed, cornerSpeed, SEARCH_ACCELERATION);
  //analogWrite(rmotorPWM, rightspeed >= 0 ? std::min(255,rightspeed) : 0); // set right motor speed
  //analogWrite(lmotorPWM, leftspeed >= 0 ? std::min(255,leftspeed) : 0); // set left motor speed

  PathRecorder::SegmentDirection currentDirection = pathRecorder.getFirstSegment();
  PathRecorder::SegmentDirection nextDirection = pathRecorder.peakNextSegment();
  
  while(!playbackRecorder.detectedEndMarker())
  {
    //photoread();
//#if SENSOR_POLAIRTY_TRUE
  //  sensdiff = sensors.get_front_diff();
//#else
  //  sensdiff = -sensors.get_front_diff();
//#endif

    // Detect and record any markers found 
    //int leftPosition = (int)(-encoder_l.count() * ENCODER_CALIBRATION);
    //int rightPosition = (int)(encoder_r.count() * ENCODER_CALIBRATION);
    //playbackRecorder.record(sensors.see_left_wall, sensors.see_right_wall, leftPosition, rightPosition);
    playbackRecorder.record(sensors.see_left_wall?1000:0, sensors.see_right_wall?1000:0, encoders.robot_distance(), encoders.robot_angle());

    // Adjust the segment number, if changed
    if(currentDirection != PathRecorder::SegmentDirection::endMark && pathRecorder.currentSegmentNumber() != playbackRecorder.currentSegmentNumber())
    {
      currentDirection = pathRecorder.getNextSegment();
      nextDirection = pathRecorder.peakNextSegment();
    }

//     // Push through PID controller
// #ifdef DETECT_CROSSOVER_TWITCH    
// #if SENSOR_POLAIRTY_TRUE
//     if(rsidesens < markerHighThreshold ||
//        lsidesens < markerHighThreshold)
// #else
//     if(rsidesens > markerLowThreshold || 
//        lsidesens > markerLowThreshold)
// #endif
//     {
//       // Not at crossover
//       pidInput = sensdiff;
//     }
//     else
//     {
//       // Ignore crossover
//       pidInput = 0;
//     }
// #else
//     pidInput = sensdiff;
// #endif    

//     float turn = steeringPID.compute();

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
        //SerialPort.print("Decel,");
      }
      else if(distanceToGo > decelDistance)
      {
        if(playbackRecorder.getCurrentSegmentDistance() < ACCELERATION_DISTANCE)
        {
          // Accelerate up to speed
          int speed = (forwardSpeed - cornerSpeed) * playbackRecorder.getCurrentSegmentDistance() / ACCELERATION_DISTANCE + cornerSpeed;
          // Keep going if already up to speed
          currentSpeed = std::max(speed, currentSpeed);
          //SerialPort.print("Accel,");
        }
        else
        {
          // Want max speed
          currentSpeed = forwardSpeed;
          //SerialPort.print("Fast,");
        }
      }
      else
      {
        currentSpeed = cornerSpeed;
        //SerialPort.print("D_done,");
      }
    }
    else
    {
      currentSpeed = cornerSpeed;
      //SerialPort.print("Corner,");

      // Add a bit of feed-forward
      // if(currentDirection == PathRecorder::rightTurn)
      //   turn -= FEED_FORWARD;
      // else if(currentDirection == PathRecorder::leftTurn)
      //   turn += FEED_FORWARD;
    }

    // Adjust the turn PID output based on new speed
    //turn = constrain(turn * (1 + (currentSpeed - 64) * PID_TURN_FACTOR), -MAX_MOTOR_VOLTS, MAX_MOTOR_VOLTS);

    //rightspeed = int(currentSpeed * (1 + turn));
    //leftspeed = int(currentSpeed * (1 - turn));
 
    motion.set_target_velocity(currentSpeed);

    //analogWrite(rmotorPWM, rightspeed >= 0 ? std::min(255,rightspeed) : 0); // set right motor speed
    //analogWrite(lmotorPWM, leftspeed >= 0 ? std::min(255,leftspeed) : 0); // set left motor speed

    // Put LED into state according to how many markers seen
    digitalWrite(indicatorLedBlue, playbackRecorder.currentSegmentNumber() % 2 ? LOW : HIGH); // toggle LED

  //  logFollowerState(playbackRecorder.currentSegmentNumber(), turn);
    
/*
    // Debug
    SerialPort.print(playbackRecorder.currentSegmentNumber());
    SerialPort.print(",");        PathRecorder::printDirection(currentDirection);
    SerialPort.print(",seg_d=");  SerialPort.print(pathRecorder.getSegmentDistance());
    SerialPort.print(",cur_d=");  SerialPort.print(playbackRecorder.getCurrentSegmentDistance());
    SerialPort.print(",s=");      SerialPort.print(currentSpeed);
    SerialPort.print(",turn=");   SerialPort.print(turn);
    SerialPort.print(",pos=");    SerialPort.print(leftPosition);
    SerialPort.print(",");        SerialPort.println(leftPosition);
*/

    count++;
    delay(2);
  }
  
  endTime = millis();

  if(!pursuitMode)
  {
    SerialPort.print("Slowdown ");
    SerialPort.println(slowdownSpeed);

    // Slowdown sequence
    // int stopMarkerPos_r = encoder_r.count();
    // while(stopMarkerPos_r + STOP_DISTANCE > encoder_r.count())
    int stopMarkerPos_r = forward.position();
    while(stopMarkerPos_r + STOP_DISTANCE > forward.position())
    {
      //photoread();

  //#if SENSOR_POLAIRTY_TRUE
//      sensdiff = sensors.get_front_diff();
//  #else
  //    sensdiff = -sensors.get_front_diff();
  //#endif

      // Push through PID controller
      //pidInput = sensdiff;
      //float turn = steeringPID.compute();

      // Adjust the turn PID output based on new speed
      //turn = constrain(turn * (1 + (slowdownSpeed - 64) * PID_TURN_FACTOR), -MAX_MOTOR_VOLTS, MAX_MOTOR_VOLTS);

      // Set the motors to the default speed +/- turn
      //rightspeed = slowdownSpeed * (1 + turn);
      //leftspeed = slowdownSpeed * (1 - turn);
      motion.set_target_velocity(slowdownSpeed);
      //analogWrite(rmotorPWM, rightspeed >= 0 ? std::min(255,rightspeed) : 0); // set right motor speed
      //analogWrite(lmotorPWM, leftspeed >= 0 ? std::min(255,leftspeed) : 0); // set left motor speed
      
    //  logFollowerState(playbackRecorder.currentSegmentNumber(), turn);
      
      delay(2);
    }
  
    digitalWrite(indicatorLedBlue, LOW); // LED off
    
    SerialPort.println(F("Line track re-run complete"));
    motion.stop_move();
    delay(100);
    while(!motion.move_finished())
    {
      delay(2);
    }
    sensors.disable();
    motion.reset_drive_system();
    motion.disable_drive();
    sensors.set_steering_mode(STEERING_OFF);
  }
  else
  {
    // Persuit mode
    SerialPort.print("Pursuit mode circuit complete");
    digitalWrite(indicatorLedBlue, LOW); // LED off
  }

  // Print results
  SerialPort.print("Time mS: "); SerialPort.println(endTime-startTime);
  SerialPort.print("Loop: "); SerialPort.print(count);
  SerialPort.print(" ("); SerialPort.print(count/((endTime-startTime)/1000.0)); SerialPort.println("/sec)"); 
  playbackRecorder.printPath();
}


void test_forward(int distance) {
    // note that changes to the speeds are likely to affect
    // the other turn parameters
    //sensors.wait_for_user_start();
    motion.reset_drive_system();
    sensors.set_steering_mode(STEERING_OFF);
    //reporter.print_justified(encoders.robot_distance(), 5);
    // move 
    motion.move(distance, SEARCH_SPEED, 0, SEARCH_ACCELERATION);
    //motion.turn(90, SEARCH_SPEED, 0, SEARCH_ACCELERATION);
    // Be sure robot has come to a halt.
    motion.stop();
    // Show actual distance measured
    //reporter.print_justified(encoders.robot_distance(), 5);
    // return to idle
    motion.reset_drive_system();
    motion.disable_drive();
    sensors.set_steering_mode(STEERING_OFF);
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
      int fnswvalue = switches.read(); // read function switch value after button released
      int fastRunSpeed = fnswvalue * FAST_LINE_FOLLOWER_SPEED_PER_SWITCH;
      if(fastRunSpeed <= basespeed)
        fastRunSpeed = basespeed + FAST_LINE_FOLLOWER_SPEED_PER_SWITCH;
      //delay(2000);

      replayRecordedPath(pathRecorder, fastRunSpeed, basespeed - CORNER_APPROACH_SLOWDOWN, basespeed, (int)(basespeed*SLOWDOWN_SPEED_RATIO), pursuitMode);
    }
}

