#include "Arduino.h"
//#include "defaults.h"
//#include "config.h"
//#include "wallFollow.h"
//#include "motors.h"
//#include "pid.h"
/*
// Globals
Motors motors;
float wall_follow_error_filtered = 0.0;
float lastDistL = 0.0;
float lastDistR = 0.0;

// Characteristics PRofiles
typedef struct  
{
  unsigned int ahead_max_speed;
  unsigned int left_leadin_distance;
  unsigned int left_leadin_distance_short;
  unsigned int left_leadin_speed;
  unsigned int left_turn_speed;
  unsigned int left_leadout_distance;
  unsigned int left_leadout_speed;
  unsigned int right_leadin_distance;
  unsigned int right_leadin_speed;
  unsigned int right_turn_speed;
  unsigned int right_leadout_distance;
  unsigned int right_leadout_speed;
  unsigned int about_leadin_distance;
  unsigned int about_leadin_speed;
  unsigned int about_turn_speed;
  unsigned int about_leadout_distance;
  unsigned int about_leadout_speed;
} MODE_PROFILE_TABLE;

static MODE_PROFILE_TABLE mode_profiles[] =
{
  // Classic maze - Green
  {
    // Ahead 
    forward_speed,
    // Left
    100, 50, // In Distances
    turn_leadin_speed, turn_speed,
    50,      // Out distance
    turn_leadout_speed,
    // Right
    40,    // In distance
    turn_leadin_speed, turn_speed,
    50,     // Out distance
    turn_leadout_speed,
    // About turn
    40,   // In distance
    turn_leadin_speed, turn_180_speed,
    20,   // Out distance
    turn_leadout_speed
  },
  // Classic maze faster - Blue
  {
    // Ahead 
    forward_speed,
    // Left
    160, 50, turn_leadin_speed * 3 / 2,
    turn_speed * 3 / 2,
    30, turn_leadout_speed * 3 / 2,
    // Right
    30, turn_leadin_speed * 3 / 2,
    turn_speed * 3 / 2,
    90, turn_leadout_speed * 3 / 2,
    // About turn
    40, turn_leadin_speed * 3 / 2,
    turn_180_speed * 3 / 2,
    5, turn_leadout_speed * 3 / 2
  }
};

// Move forward fixed distance
void forward(int distance, int speed)
{
  // Move forward until distance reached or too near the forward wall
  float start_pos_r = encoder_r.count() * encode_calibrate_r;
  if(distance > 0)
  {
    motors.set_left_motor_volts(speed);
    motors.set_right_motor_volts(speed);
    while(encoder_r.count() * encode_calibrate_r - start_pos_r < distance &&
          sensorForward() < wall_follow_forward_min_distance)
    {
      delay(loop_speed_ms);
      //SerialPort.print("FORWARD ");
      //SerialPort.print(encoder_r.count() * encode_calibrate_l);  SerialPort.print("mm ");
      //SerialPort.println();
      photoread(true);
      logSensors("FORWARD");
    }
  }
  else
  {
    motors.set_left_motor_volts(-speed);
    motors.set_right_motor_volts(-speed);
    while(encoder_r.count() * encode_calibrate_r - start_pos_r > distance)
    {
      delay(loop_speed_ms);
      //SerialPort.print("FORWARD ");
      //SerialPort.print(encoder_r.count() * encode_calibrate_l);  SerialPort.print("mm ");
      //SerialPort.println();
      photoread(true);
      logSensors("REVERSE");
    }
  }
}

float get_wall_follow_error()
{
  static const float interval = LOOP_INTERVAL;

  // Simple P(I)D controller using the nearest wall
  float newDistL = sensorLeft();
  float newDistR = sensorRight();
  float error;
  if(newDistL > newDistR)
    error = (newDistL - wall_follow_left_distance) * kp + (newDistL - lastDistL) * kd / interval;
  else if(newDistR > wall_follow_left_distance_min)
    error = -((newDistR - wall_follow_left_distance) * kp + (newDistR - lastDistR) * kd / interval);
  else
    // Clamp error if reached a potential gap
    error = 0.0;

  // Simple filter
  wall_follow_error_filtered = wall_follow_error_filtered * (1.0-wall_sensor_filter_ratio) + error * wall_sensor_filter_ratio;

  lastDistL = newDistL;
  lastDistR = newDistR;

  return wall_follow_error_filtered;
}

void reset_PID()
{
  lastDistL = sensorLeft();
  lastDistR = sensorRight();
  wall_follow_error_filtered = 0.0;
}

void forward_with_wall_follow(int distance, int speed)
{
  // Move forward until distance reached or too near the forward wall
  float start_pos_r = encoder_r.count() * encode_calibrate_r;
  while(encoder_r.count() * encode_calibrate_r - start_pos_r < distance &&
        sensorForward() < wall_follow_forward_min_distance)
  {
      delay(loop_speed_ms);
      photoread(true);
      motors.set_left_motor_volts(speed - get_wall_follow_error());
      motors.set_right_motor_volts(speed + get_wall_follow_error());
      logSensors("FORWARD-FOLLOW");
  }
}

void turn_left_90(int speed)
{
//  motors.stop();
//  delay(500);
 
  // Rotate 90 degrees about left wheel
  float start_pos_l = encoder_l.count() * encode_calibrate_l;
  float start_pos_r = encoder_r.count() * encode_calibrate_r;
  motors.set_left_motor_volts(0);
  motors.set_right_motor_volts(speed);

  int required_dist_r = (int)(3.14159/2.0 * turning_diameter_mm * turn_left_angle_inertia_compensation);
  
  while(encoder_r.count() * encode_calibrate_r - start_pos_r < 
        required_dist_r + (encoder_l.count() * encode_calibrate_l - start_pos_l))
  {
    delay(loop_speed_ms);
    //SerialPort.print("LEFT ");
    //SerialPort.print(encoder_r.count() * encode_calibrate_l);  SerialPort.print("mm ");
    //SerialPort.print(encoder_l.count() * encode_calibrate_r);  SerialPort.print("mm ");
    //SerialPort.println();
    photoread(true);
    logSensors("LEFT90");
  }

  motors.stop();
  delay(400);
}

void turn_right_90(int speed)
{
//  motors.stop();
//  delay(500);
  // Rotate 90 degrees about right wheel
  float start_pos_l = encoder_l.count() * encode_calibrate_l;
  float start_pos_r = encoder_r.count() * encode_calibrate_r;
  motors.set_left_motor_volts(speed);
  motors.set_right_motor_volts(0);

  int required_dist_l = (int)(3.14159/2.0 * turning_diameter_mm * turn_right_angle_inertia_compensation);
  
  while(encoder_l.count() * encode_calibrate_l - start_pos_l < 
        required_dist_l + (encoder_r.count() * encode_calibrate_r - start_pos_r))
  {
    delay(loop_speed_ms);
    //SerialPort.print("RIGHT ");
    //SerialPort.print(encoder_r.count() * encode_calibrate_l);  SerialPort.print("mm ");
    //SerialPort.print(encoder_l.count() * encode_calibrate_r);  SerialPort.print("mm ");
    //SerialPort.println();
    photoread(true);
    logSensors("RIGHT90");
  }

  motors.stop();
  delay(400);
}

void turn_right_180(int speed)
{
  motors.stop();
  delay(100);

  // Rotate 180 degrees about centre axis
  float start_pos_l = encoder_l.count() * encode_calibrate_l;
  float start_pos_r = encoder_r.count() * encode_calibrate_r;
  motors.set_left_motor_volts(-speed);
  motors.set_right_motor_volts(speed);

  int required_dist_total = (int)(3.14159 * turning_diameter_mm * turn_right180_angle_inertia_compensation);
  
  while((encoder_l.count() * encode_calibrate_l - start_pos_l) - (encoder_r.count() * encode_calibrate_r - start_pos_r) <
        required_dist_total)
  {
    delay(loop_speed_ms);
    photoread(true);
    logSensors("RIGHT180");
  }

  motors.stop();
  delay(500);
}


void logSensors(const char *mode)
{
  static bool firstLog = true;
  if(firstLog)
  {
    firstLog = false;
    // Print header
    SerialPort.print("Mode,LeftCal,FrontCal,RightCal,EncLCal,EncRCal");
#ifdef LOG_RAW_SENSORS  
    SerialPort.println(",LeftRaw,FrontRaw,RightRaw,EncLRaw,EncRRaw");
#else
    SerialPort.println();
#endif
  }

  SerialPort.print(mode);            SerialPort.print(",");
  
  // Sensors
  SerialPort.print(sensorLeft());     SerialPort.print(",");
  SerialPort.print(sensorForward());  SerialPort.print(",");
  SerialPort.print(sensorRight());    SerialPort.print(",");
  
  // Encoders
  SerialPort.print(encoder_l.count() * encode_calibrate_l);  SerialPort.print("mm,");
  SerialPort.print(encoder_r.count() * encode_calibrate_r);  SerialPort.print("mm,");

#ifdef LOG_RAW_SENSORS  
  SerialPort.print(sensors.left().getRaw());       SerialPort.print(",");
  SerialPort.print(sensors.frontLeft().getRaw());  SerialPort.print(",");
  SerialPort.print(sensors.frontRight().getRaw()); SerialPort.print(",");
  SerialPort.print(sensors.right().getRaw());      SerialPort.print(",");
  
  // Encoders
  SerialPort.print(encoder_l.count());             SerialPort.print(",");
  SerialPort.print(encoder_r.count());
#endif

  SerialPort.println();
}

void testMovesForCalibrate()
{
/*
  // Zero encoders
  encoder_l.reset_count();
  encoder_r.reset_count();

  // Forward 500mm
  forward(500, 64);

  // Stop
  motors.stop();
  delay(500);

  // Display results
  logSensors("FORWARD 500mm");

  long f500_pos_l = encoder_l.count();
  long f500_pos_r = encoder_r.count();

  SerialPort.print("FORWARD 500mm: ");
  SerialPort.print(f500_pos_l * encode_calibrate_l);  SerialPort.print("mm,");
  SerialPort.print(f500_pos_r * encode_calibrate_r);  SerialPort.println("mm");
  
  // Wait for button
  buttonwait(50); // wait for function button to be pressed
  delay(1000);
*/
/*
  // Zero encoders
  encoder_l.reset_count();
  encoder_r.reset_count();

  // Forward 500mm
  //for(int i = 0; i <100; i++)
  //  forward_with_wall_follow(5, 64);
  forward(500, 64);
  long f500_1_pos_l = encoder_l.count();
  long f500_1_pos_r = encoder_r.count();

//  motors.stop();
//  delay(100);

  // Zero encoders
  encoder_l.reset_count();
  encoder_r.reset_count();

  turn_left_90(32);
  long l90_2_pos_l = encoder_l.count();
  long l90_2_pos_r = encoder_r.count();

 // motors.stop();
 // delay(400);

  // Zero encoders
  encoder_l.reset_count();
  encoder_r.reset_count();

  forward(500, 64);

  long f500_3_pos_l = encoder_l.count();
  long f500_3_pos_r = encoder_r.count();

//  motors.stop();
//  delay(100);

  // Zero encoders
  encoder_l.reset_count();
  encoder_r.reset_count();

  turn_right_90(32);
  long r90_4_pos_l = encoder_l.count();
  long r90_4_pos_r = encoder_r.count();

//  motors.stop();
//  delay(400);

  encoder_l.reset_count();
  encoder_r.reset_count();

  forward(500, 64);
//  motors.stop();
//  delay(500);
  encoder_l.reset_count();
  encoder_r.reset_count();
  turn_right_180(32);
//  motors.stop();
//  delay(500);
  encoder_l.reset_count();
  encoder_r.reset_count();
  forward(500, 64);

  // Stop
  // Zero encoders
  encoder_l.reset_count();
  encoder_r.reset_count();
  motors.stop();

  delay(500);
  
  // Display results
  logSensors("FORWARD 500mm/LEFT 90/Forward 500mm");

  long f500_l90_f500_pos_l = encoder_l.count();
  long f500_l90_f500_pos_r = encoder_r.count();

  SerialPort.print("FORWARD 500mm/LEFT 90/Forward 500mm: ");
  SerialPort.print(f500_l90_f500_pos_l * encode_calibrate_l);  SerialPort.print("mm, ");
  SerialPort.print(f500_l90_f500_pos_r * encode_calibrate_r);  SerialPort.println("mm");

  // Wait for button
  buttonwait(50); // wait for function button to be pressed

  // Display again
/*  
  SerialPort.print("FORWARD 500mm: ");
  SerialPort.print(f500_pos_l * encode_calibrate_l);  SerialPort.print("mm,");
  SerialPort.print(f500_pos_r * encode_calibrate_r);  SerialPort.println("mm");
  */
/*
  SerialPort.print("FORWARD 500mm/LEFT 90/Forward 500mm: ");
  SerialPort.print(f500_1_pos_l * encode_calibrate_l);  SerialPort.print("mm, ");
  SerialPort.print(f500_1_pos_r * encode_calibrate_r);  SerialPort.println("mm");
  SerialPort.print(l90_2_pos_l * encode_calibrate_l);  SerialPort.print("mm, ");
  SerialPort.print(l90_2_pos_r * encode_calibrate_r);  SerialPort.println("mm");
  SerialPort.print(f500_3_pos_l * encode_calibrate_l);  SerialPort.print("mm, ");
  SerialPort.print(f500_3_pos_r * encode_calibrate_r);  SerialPort.println("mm");
  SerialPort.print(f500_l90_f500_pos_l * encode_calibrate_l);  SerialPort.print("mm, ");
  SerialPort.print(f500_l90_f500_pos_r * encode_calibrate_r);  SerialPort.println("mm");
  
}

// Follow the left wall, return TRUE if successful
bool FollowLeftWall()
{
    // Wait for button
    buttonwait(50); // wait for function button to be pressed

    // Ensure all LEDs off
    digitalWrite(sensorLED1, 0);
    digitalWrite(sensorLED2, 0);
    digitalWrite(indicatorLedBlue, 0);

    delay(1000);
 
    //testMovesForCalibrate();
    //return false;

    // Wall follower approach
    // 1) Forward until gap seen on left or blocked ahead
    // 2) Continue to centre of cell (10cm)
    // 3) If gap to left, turn 90deg left then forward 10cm
    // 4) If blocked ahead and open to right, turn 90deg right then foward 10cm
    // 5) else about turn

    // Go
    bool justTurned = false;
    int mode = 0;

    // Reset PID
    reset_PID();

    SerialPort.print("FollowLeftWall: speed="); SerialPort.println(mode_profiles[mode].left_leadin_speed);

    while(true) 
    {
        photoread(true);
//        sensors.waitForSample();
        
        // Gap on left?
        if(sensorLeft() < wall_follow_left_gap_threshold)
        {
          logSensors("GAP LEFT");
          digitalWrite(sensorLED2, 1);
  
          // Move past the gap, so can turn
          if(!justTurned)
          {
            // Need to move wheels into the gap
            forward(mode_profiles[mode].left_leadin_distance, mode_profiles[mode].left_leadin_speed);
          }
          else
          {
            // Wheels already in gap, just need a bit of clearance
            forward(mode_profiles[mode].left_leadin_distance_short, mode_profiles[mode].left_leadin_speed);
          }          

          // Turn
          turn_left_90(mode_profiles[mode].left_turn_speed);

          digitalWrite(sensorLED1, 1);

          // Straighten up
          reset_PID();
          forward(mode_profiles[mode].left_leadout_distance, mode_profiles[mode].left_leadout_speed);
          
          digitalWrite(sensorLED1, 0);
          digitalWrite(sensorLED2, 0);

          // Reset PID
//          reset_PID();
          justTurned = true;
        }
        else 
        // Blocked ahead - need to turn right or u-turn?
        if(sensorForward() > wall_follow_ahead_blocked_threshold)
        {
          //SerialPort.println(" BLOCKED AHEAD");
          // Do we need to do 90 degree or 180 degree?
          if(sensorRight() < wall_follow_right_gap_threshold)
          {
            // Ok to do 90 degree
            logSensors("GAP RIGHT");
            digitalWrite(sensorLED1, 1);

  //motors.stop();
  //delay(500);
            forward(mode_profiles[mode].right_leadin_distance, mode_profiles[mode].right_leadin_speed); // Will stop if gets too close

            turn_right_90(mode_profiles[mode].right_turn_speed);

            digitalWrite(sensorLED2, 1);

            // We will have a left wall, follow it
            reset_PID();
            forward_with_wall_follow(mode_profiles[mode].right_leadout_distance, mode_profiles[mode].right_leadout_speed);

            digitalWrite(sensorLED2, 0);
            digitalWrite(sensorLED1, 0);
          }
          else
          {
            // Need to do 180 degree
            logSensors("CUL-DE-SAC");
            digitalWrite(indicatorLedBlue, 1);

  //motors.stop();
  //delay(500);

            forward_with_wall_follow(mode_profiles[mode].about_leadin_distance, mode_profiles[mode].about_leadin_speed); // Will stop if gets too close

            turn_right_180(mode_profiles[mode].about_turn_speed);

            digitalWrite(sensorLED1, 1);
            digitalWrite(sensorLED2, 1);

            // We will have a left wall, follow it
            reset_PID();
            forward_with_wall_follow(mode_profiles[mode].about_leadout_distance, mode_profiles[mode].about_leadout_speed);

            digitalWrite(sensorLED1, 0);
            digitalWrite(sensorLED2, 0);
            digitalWrite(indicatorLedBlue, 0);
          }

          // Reset PID
//          reset_PID();
          justTurned = true;
        }
        else
        {
          // Continue ahead
          logSensors("AHEAD");
          forward_with_wall_follow(5, mode_profiles[mode].ahead_max_speed);
          //motors.turn(mode_profiles[mode].ahead_max_speed, -get_wall_follow_error());
          justTurned = false;
        }
    }

    // Success
    return true;
}
*/

// Simple wall-follower function
//
void simpleWallFollower(int basespeed) 
{
}
/*
  int sensdiff = 0;
  int leftTurnCount = 0;

  SerialPort.print("Wall Follower, speed: "); SerialPort.println(basespeed);
  
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
  digitalWrite(rmotorDIR, HIGH); // set right motor forward
  digitalWrite(lmotorDIR, LOW); // set left motor forward

  // Forward to start line
  rightspeed = basespeed;
  leftspeed = basespeed;
  analogWrite(rmotorPWM, rightspeed); // set right motor speed
  analogWrite(lmotorPWM, leftspeed); // set left motor speed

  while(true)
  {
    photoread(true);
    sensdiff = lfrontsens - wallFollowerTargetDistance;
    bool forwardBlocked = rfrontsens > wallFollowerForwardAvoidDistance;
    bool leftGap = lfrontsens < wallFollowerLeftGapThreshold;

    static float sensdiffFitered;
    sensdiffFitered = sensdiffFitered * 0.5 + sensdiff * 0.5;

    //SerialPort.println(lfrontsens);
    //SerialPort.println(sensdiff);

    // Push through PID controller
    pidInput = sensdiffFitered;
    float turn = steeringPID.compute();

    // Set the motors to the default speed +/- turn
    if(!leftGap)
    {
      if(!forwardBlocked)
      {
        // Limit the turn to 20%
        turn = std::max(std::min(turn, 0.2F), -0.2F);
        // Keep on following left wall
        rightspeed = int(basespeed * (1 + turn));
        leftspeed = int(basespeed * (1 - turn));

        // We've seen a wall, reset the coast counter
        leftTurnCount = 0;

        digitalWrite (sensorLED1, LOW);  // Right/Red LED
        digitalWrite (sensorLED2, LOW);   // Left/Green LED
        digitalWrite (indicatorLedBlue, LOW);  // Centre/Blue LED
      }
      else
      {
        // Blocked ahead - turn right
        rightspeed = -int(basespeed * 0.8);
        leftspeed = int(basespeed * 0.9);

        // May need a very short turn
        leftTurnCount = wallFollowerLeftTurnDelay;

        digitalWrite (sensorLED1, HIGH);  // Right/Red LED
        digitalWrite (sensorLED2, LOW);   // Left/Green LED
        digitalWrite (indicatorLedBlue, LOW);  // Centre/Blue LED
      }
    }
    else if(++leftTurnCount <= wallFollowerLeftTurnDelay)
    {
      // Gap on left, but keep going ahead a small amount first
      // slightly right
      rightspeed = int(basespeed * 0.95);
      leftspeed = int(basespeed);

      digitalWrite (sensorLED1, LOW);  // Right/Red LED
      digitalWrite (sensorLED2, HIGH);   // Left/Green LED
      digitalWrite (indicatorLedBlue, HIGH);  // Centre/Blue LED
    }
    else
    {
      // Gap on left, turn into it now
      rightspeed = int(basespeed * 1.3);
      leftspeed = int(basespeed * 0.2);

      digitalWrite (sensorLED1, LOW);  // Right/Red LED
      digitalWrite (sensorLED2, HIGH);   // Left/Green LED
      digitalWrite (indicatorLedBlue, LOW);  // Centre/Blue LED
    }

    // Update motors
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

    delay(3);
  }
}

*/