#include "Arduino.h"
#include "wallFollow.h"
#include "motors.h"
#include "defaults.h"

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
    160, 50, turn_leadin_speed,
    turn_speed,
    100, turn_leadout_speed,
    // Right
    160, turn_leadin_speed,
    turn_speed,
    100, turn_leadout_speed,
    // About turn
    40, turn_leadin_speed,
    turn_180_speed,
    20, turn_leadout_speed
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
    motors.forwardPower(speed);
    while(encoder_r.count() * encode_calibrate_r - start_pos_r < distance &&
          sensorForward() < wall_follow_forward_min_distance)
    {
      delay(10);
      //Serial.print("FORWARD ");
      //Serial.print(encoder_r.count() * encode_calibrate_l);  Serial.print("mm ");
      //Serial.println();
      photoread();
      logSensors("FORWARD");
    }
  }
  else
  {
    motors.forwardPower(-speed);
    while(encoder_r.count() * encode_calibrate_r - start_pos_r > distance)
    {
      delay(10);
      //Serial.print("FORWARD ");
      //Serial.print(encoder_r.count() * encode_calibrate_l);  Serial.print("mm ");
      //Serial.println();
      photoread();
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
    error = (newDistL - wall_follow_left_distance) * kp + (newDistL - lastDistL) * kd * interval;
  else if(newDistR > wall_follow_left_distance_min)
    error = -((newDistR - wall_follow_left_distance) * kp + (newDistR - lastDistR) * kd * interval);
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
      logSensors("FORWARD-FOLLOW");
      motors.turn(speed, -get_wall_follow_error());
      photoread();
  }
}

void turn_left_90(int speed)
{
//  motors.stop();
//  delay(500);
 
  // Rotate 90 degrees
  float start_pos_l = encoder_l.count() * encode_calibrate_l;
  float start_pos_r = encoder_r.count() * encode_calibrate_r;
  motors.turn(speed, speed);

  int required_dist_r = (int)(3.14159/2.0 * turning_diameter_mm) - turn_angle_inertia_compensation;
  
  while(encoder_r.count() * encode_calibrate_r - start_pos_r < 
        (encoder_l.count() * encode_calibrate_l - start_pos_l + required_dist_r))
  {
    delay(10);
    //Serial.print("LEFT ");
    //Serial.print(encoder_r.count() * encode_calibrate_l);  Serial.print("mm ");
    //Serial.print(encoder_l.count() * encode_calibrate_r);  Serial.print("mm ");
    //Serial.println();
    photoread();
    logSensors("LEFT90");
  }
//  motors.stop();
//  delay(500);
}

void turn_right_90(int speed)
{
//  motors.stop();
//  delay(500);
  // Rotate 90 degrees
  float start_pos_l = encoder_l.count() * encode_calibrate_l;
  float start_pos_r = encoder_r.count() * encode_calibrate_r;
  motors.turn(speed, -speed);

  int required_dist_l = (int)(3.14159/2.0 * turning_diameter_mm) - turn_angle_inertia_compensation;
  
  while(encoder_l.count() * encode_calibrate_l - start_pos_l < 
        (encoder_r.count() * encode_calibrate_r - start_pos_r + required_dist_l))
  {
    delay(10);
    //Serial.print("RIGHT ");
    //Serial.print(encoder_r.count() * encode_calibrate_l);  Serial.print("mm ");
    //Serial.print(encoder_l.count() * encode_calibrate_r);  Serial.print("mm ");
    //Serial.println();
    photoread();
    logSensors("RIGHT90");
  }
//  motors.stop();
//  delay(500);
}

void turn_right_180(int speed)
{
//  motors.stop();
//  delay(500);
  // Rotate 90 degrees
  float start_pos_l = encoder_l.count() * encode_calibrate_l;
  float start_pos_r = encoder_r.count() * encode_calibrate_r;
  motors.turn(0, -speed);

  int required_dist_l = (int)(3.14159 * turning_diameter_mm) - turn_angle_inertia_compensation;
  
  while(encoder_l.count() * encode_calibrate_l - start_pos_l <
        (encoder_r.count() * encode_calibrate_r - start_pos_r + required_dist_l))
  {
    delay(10);
    photoread();
    logSensors("RIGHT180");
  }
//  motors.stop();
//  delay(500);
}


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

// Follow the left wall, return TRUE if successful
bool FollowLeftWall()
{
    // Wall follower approach
    // 1) Forward until gap seen on left or blocked ahead
    // 2) Continue to centre of cell (10cm)
    // 3) If gap to left, turn 90deg left then forward 10cm
    // 4) If blocked ahead and open to right, turn 90deg right then foward 10cm
    // 5) else about turn

    delay(500);

    // Go
    bool justTurned = false;
    int mode = 0;

    // Reset PID
    reset_PID();

    Serial.print("FollowLeftWall: speed="); Serial.println(mode_profiles[mode].left_leadin_speed);

    while(true) 
    {
        photoread();
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
          turn_left_90(mode_profiles[mode].left_leadin_speed);

          // Straighten up
          reset_PID();
          forward(mode_profiles[mode].left_leadout_distance, mode_profiles[mode].left_leadout_speed);
          
          digitalWrite(sensorLED2, 0);

          // Reset PID
//          reset_PID();
          justTurned = true;
        }
        else 
        // Blocked ahead - need to turn right or u-turn?
        if(sensorForward() > wall_follow_ahead_blocked_threshold)
        {
          //Serial.println(" BLOCKED AHEAD");
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

            // We will have a left wall, follow it
            reset_PID();
            forward_with_wall_follow(mode_profiles[mode].right_leadout_distance, mode_profiles[mode].right_leadout_speed);

            digitalWrite(sensorLED1, 0);
          }
          else
          {
            // Need to do 180 degree
            logSensors("CUL-DE-SAC");
            digitalWrite(indicatorLedBlue, 1);

  //motors.stop();
  //delay(500);

            forward(mode_profiles[mode].about_leadin_distance, mode_profiles[mode].about_leadin_speed); // Will stop if gets too close
            turn_right_180(mode_profiles[mode].about_turn_speed);

            // We will have a left wall, follow it
            reset_PID();
            forward_with_wall_follow(mode_profiles[mode].about_leadout_distance, mode_profiles[mode].about_leadout_speed);

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
