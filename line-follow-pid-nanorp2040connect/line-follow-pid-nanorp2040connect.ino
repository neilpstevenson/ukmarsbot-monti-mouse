#include <WiFiNINA.h>
#include "defaults.h"
#include "ukmarsbot-pins.h"

int basespeed = MIN_BASE_SPEED; //Base speed (constant)

#include "pid.h"
#include "debounce.h"
#include "quadrature.h"
#include "pathRecorder.h"

//Inputs
int rfrontsens = 0; //Right front sensor value
int lfrontsens = 0; //Left front sensor value
int rsidesens = 0; //Right side sensor value
int lsidesens = 0; //Left side sensor value
int sensdiff = 0; //Difference between front sensors
int batteryvolts = 0; // battery voltage reading
int batterycalc = 0; // working field
int switchvoltage = 0; // analogue value coming back from reading function or 4 way switch
int fnswvalue = 0; // value (in range 0 to 16) of 4 way function switch
int posn = 0; // if on line or off it and which side
unsigned long int endTime = 0;
unsigned long int startTime = 0;

//Motor variables
int rightspeed = 0; //Right motor speed
int leftspeed = 0; //Left motor speed

// Encoders
Quadrature_encoder<m1encoder2, m1encoder1> encoder_l;
Quadrature_encoder<m2encoder2, m2encoder1> encoder_r;

PathRecorder pathRecorder;

// Initialise hardware
void setup() 
{
  Serial.begin(115200);

  pinMode(lmotorDIR, OUTPUT);
  pinMode(rmotorDIR, OUTPUT);
  pinMode(lmotorPWM, OUTPUT);
  pinMode(rmotorPWM, OUTPUT);
  pinMode(sensorLED1, OUTPUT);
  pinMode(sensorLED2, OUTPUT);
  pinMode(trigger, OUTPUT);
  pinMode(indicatorLedBlue, OUTPUT);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  
  stopmotors(); // switch off the motors

  encoder_l.begin(pull_direction::up, resolution::quarter);
  encoder_r.begin(pull_direction::up, resolution::quarter);

}

// Wait for button to be pressed and released
void buttonwait(int period)
{ 
  // waits until tactile button is pressed
  digitalWrite(indicatorLedBlue, HIGH); // put LED on
  switchvoltage = analogRead(fourwayswitch);
  int flash = 0;
  while (switchvoltage < 1000)
  { 
      delay(10);
      // while button not pressed
      switchvoltage = analogRead(fourwayswitch);

      if(++flash % period == 0)
        digitalWrite(indicatorLedBlue, LOW); // put LED off
      else if(flash % period == period/2)
        digitalWrite(indicatorLedBlue, HIGH); // put LED on
  }
  
  digitalWrite(indicatorLedBlue, LOW); // LED off
  // Debounce time
  delay(20);
  while (switchvoltage >= 1000)
  { 
      // while button pressed
      switchvoltage = analogRead(fourwayswitch);
  }
  digitalWrite(indicatorLedBlue, HIGH); // put LED on
  // Time for finger to be removed...
  delay(500);
}

void functionswitch()
{
  switchvoltage = analogRead(fourwayswitch);
  fnswvalue = 16;
  if (switchvoltage > 15) fnswvalue = fnswvalue - 1;
  if (switchvoltage > (15+29)) fnswvalue = fnswvalue - 1;
  if (switchvoltage > (15+108)) fnswvalue = fnswvalue - 1;
  if (switchvoltage > (15+172)) fnswvalue = fnswvalue - 1;
  if (switchvoltage > (15+266)) fnswvalue = fnswvalue - 1;
  if (switchvoltage > (15+324)) fnswvalue = fnswvalue - 1;
  if (switchvoltage > (15+389)) fnswvalue = fnswvalue - 1;
  if (switchvoltage > (15+443)) fnswvalue = fnswvalue - 1;
  if (switchvoltage > (15+547)) fnswvalue = fnswvalue - 1;
  if (switchvoltage > (15+594)) fnswvalue = fnswvalue - 1;
  if (switchvoltage > (15+650)) fnswvalue = fnswvalue - 1;
  if (switchvoltage > (15+694)) fnswvalue = fnswvalue - 1; 
  if (switchvoltage > (15+760)) fnswvalue = fnswvalue - 1;
  if (switchvoltage > (15+802)) fnswvalue = fnswvalue - 1; 
  if (switchvoltage < (15+849)) fnswvalue = fnswvalue - 1;
  if (switchvoltage >= (15+849)) fnswvalue = 0;
  if (switchvoltage > 999) fnswvalue = 16; 
}

void stopmotors() 
{
 digitalWrite(lmotorDIR, LOW); // set left motor direction
 digitalWrite(rmotorDIR, HIGH); // set right motor direction
 rightspeed = 0; // set the required right motor speed in range 0 to 255
 leftspeed = 0; // set the required left motor speed in range 0 to 255
 analogWrite(rmotorPWM, rightspeed); // set right motor speed
 analogWrite(lmotorPWM, leftspeed); // set left motor speed

 digitalWrite(trigger, LOW);  // Sensor illumination LEDs

}

void batterycheck()
{ 
  // function to check battery voltage is over 6 volts
  int batteryread = analogRead(battery); // read battery voltage
  if (batteryread > 614) 
    return; // check if over 6 volts and return if it is
  analogWrite(rmotorPWM, 0); // set right motor speed to stop
  analogWrite(lmotorPWM, 0); // set left motor speed to stop
  while (batteryread < 614)
  {
    // Flash all LEDs
    digitalWrite(indicatorLedBlue, HIGH);
    digitalWrite(sensorLED1, LOW);
    digitalWrite(sensorLED2, LOW);
    delay (100); // wait 1/10 second
    digitalWrite(indicatorLedBlue, LOW);
    digitalWrite(sensorLED1, HIGH);
    digitalWrite(sensorLED2, HIGH);
    delay (100); // wait 1/2 second
  }
} // end of batterycheck function

void photoread()
{
  // read both line sensors & start/stop sensor
  int lfrontsensAmbient = analogRead(lfront);
  int rfrontsensAmbient = analogRead(rfront);
  int rsidesensAmbient = analogRead(rside);
  int lsidesensAmbient = analogRead(lside);

  digitalWrite(trigger, HIGH);  // Sensor illumination LEDs on

  // Settling time
  delayMicroseconds(100);

  // read both line sensors & start/stop sensor
  lfrontsens = analogRead(lfront);
  rfrontsens = analogRead(rfront);
  rsidesens = analogRead(rside);
  lsidesens = analogRead(lside);

  digitalWrite(trigger, LOW);  // Sensor illumination LEDs off

// Remove ambient light level
#if SENSOR_POLAIRTY_TRUE
  lfrontsens = lfrontsens > lfrontsensAmbient ? lfrontsens - lfrontsensAmbient : 0;
  rfrontsens = rfrontsens > rfrontsensAmbient ? rfrontsens - rfrontsensAmbient : 0;
  rsidesens = rsidesens > rsidesensAmbient ? rsidesens - rsidesensAmbient : 0;
  lsidesens = lsidesens > lsidesensAmbient ? lsidesens - lsidesensAmbient : 0;
#else
  lfrontsens = lfrontsens < lfrontsensAmbient ? lfrontsens + (1023 - lfrontsensAmbient) : 1023;
  rfrontsens = rfrontsens < rfrontsensAmbient ? rfrontsens + (1023 - rfrontsensAmbient) : 1023;
  rsidesens = rsidesens < rsidesensAmbient ? rsidesens + (1023 - rsidesensAmbient) : 1023;
  lsidesens = lsidesens < lsidesensAmbient ? lsidesens + (1023 - lsidesensAmbient) : 1023;
#endif

  // light the right hand sensor LED if white line seen by left sensor
  if (rfrontsens < sensorthreshold)
  {
    digitalWrite (sensorLED1, HIGH);
  }
  else 
  {
    digitalWrite (sensorLED1, LOW);
  }
  // light the left hand sensor LED if white line seen by left sensor
  if (lfrontsens < sensorthreshold)
  {
    digitalWrite (sensorLED2, HIGH);
  }
  else 
  {
    digitalWrite (sensorLED2, LOW);
  }

//#define DEBUG_SENS
#ifdef DEBUG_SENS
  Serial.print("SENS,");
  Serial.print(lsidesens);
  Serial.print(",");
  Serial.print(lfrontsens);
  Serial.print(",");
  Serial.print(rfrontsens);
  Serial.print(",");
  Serial.println(rsidesens);
#endif
}

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
    int turn = (int)steeringPID.compute();

    // Detect and record any markers found 
    pathRecorder.record(lsidesens, rsidesens, (int)(-encoder_l.count() * ENCODER_CALIBRATION), (int)(encoder_r.count() * ENCODER_CALIBRATION));

    // Set the motors to the default speed +/- turn
    rightspeed = basespeed * (1 + turn);
    leftspeed = basespeed * (1 - turn);

    analogWrite(rmotorPWM, rightspeed >= 0 ? rightspeed : 0); // set right motor speed
    analogWrite(lmotorPWM, leftspeed >= 0 ? leftspeed : 0); // set left motor speed

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
    pidInput = sensdiff;
    int turn = (int)steeringPID.compute();

    // Set the motors to the default speed +/- turn
    rightspeed = slowdownSpeed * (1 + turn);
    leftspeed = slowdownSpeed * (1 - turn);
    analogWrite(rmotorPWM, rightspeed); // set right motor speed
    analogWrite(lmotorPWM, leftspeed); // set left motor speed
    
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
void replayRecordedPath(int forwardSpeed, int cornerSpeed, int slowdownSpeed)
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
  analogWrite(rmotorPWM, rightspeed); // set right motor speed
  analogWrite(lmotorPWM, leftspeed); // set left motor speed

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
    pidInput = sensdiff;
    int turn = (int)steeringPID.compute();

    // Set the motors to the default speed +/- turn
    if(PathRecorder::isDirectionForward(currentDirection))
    {
      int distanceToGo = pathRecorder.getSegmentDistance() - playbackRecorder.getCurrentSegmentDistance() - DECELERATION_DISTANCE;
      if(PathRecorder::isDirectionForward(nextDirection))
      {
        distanceToGo += pathRecorder.getNextSegmentDistance();
      }
      int decelDistance = forwardSpeed * CORNER_APPROACH_DISTANCE;
      if(distanceToGo > 0 && distanceToGo <= decelDistance) 
      {
        int speed = (forwardSpeed - cornerSpeed) * distanceToGo / decelDistance + cornerSpeed;
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
    }
    
    rightspeed = currentSpeed * (1 + turn);
    leftspeed = currentSpeed * (1 - turn);

    analogWrite(rmotorPWM, rightspeed >= 0 ? rightspeed : 0); // set right motor speed
    analogWrite(lmotorPWM, leftspeed >= 0 ? leftspeed : 0); // set left motor speed

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
    int turn = (int)steeringPID.compute();

    // Set the motors to the default speed +/- turn
    rightspeed = slowdownSpeed * (1 + turn);
    leftspeed = slowdownSpeed * (1 - turn);
    analogWrite(rmotorPWM, rightspeed); // set right motor speed
    analogWrite(lmotorPWM, leftspeed); // set left motor speed
    
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


void sensorTest()
{
  while(1)
  {
    // read the value from the sensor:
    int a0valu = analogRead(lside);    
    int a1valu = analogRead(lfront);    
    int a2valu = analogRead(rfront);    
    int a3valu = analogRead(rside);    
  
    digitalWrite(trigger, HIGH);  // Sensor illumination LEDs on
    delay(1);
    
    // read the value from the sensor:
    int a0val = analogRead(lside);    
    int a1val = analogRead(lfront);    
    int a2val = analogRead(rfront);    
    int a3val = analogRead(rside);    

    // Read function switch
    functionswitch();

// Remove ambient light level
#if SENSOR_POLAIRTY_TRUE
  int lsidesens = a0val > a0valu ? a0val - a0valu : 0;
  int lfrontsens = a1val > a1valu ? a1val - a1valu : 0;
  int rfrontsens = a2val > a2valu ? a2val - a2valu : 0;
  int rsidesens = a3val > a3valu ? a3val - a3valu : 0;
#else
  int lsidesens = a0val < a0valu ? a0val + (1023 - a0valu) : 1023;
  int lfrontsens = a1val < a1valu ? a1val + (1023 - a1valu) : 1023;
  int rfrontsens = a2val < a2valu ? a2val + (1023 - a2valu) : 1023;
  int rsidesens = a3val < a3valu ? a3val + (1023 - a3valu) : 1023;
#endif

    Serial.print(a0val);
    Serial.print(",");
    Serial.print(a0valu);
    Serial.print(",");
    Serial.print(lsidesens);    
    Serial.print(", ");
    Serial.print(a1val);
    Serial.print(",");
    Serial.print(a1valu);
    Serial.print(",");
    Serial.print(lfrontsens);    
    Serial.print(", ");
    Serial.print(a2val);
    Serial.print(",");
    Serial.print(a2valu);
    Serial.print(",");
    Serial.print(rfrontsens);    
    Serial.print(", ");
    Serial.print(a3val);
    Serial.print(",");
    Serial.print(a3valu);
    Serial.print(",");
    Serial.print(rsidesens);    
    Serial.print(", ");
    Serial.print(rfrontsens - lfrontsens);
    Serial.print(",");
    Serial.println(fnswvalue);

    digitalWrite(trigger, LOW);  // Sensor illumination LEDs off
    delay(20); 
  }
}

void loop() 
{
  // Get the base speed from the DIP switches
  buttonwait(50); // wait for function button to be pressed
  functionswitch(); // read function switch value after button released
  basespeed = fnswvalue * 17;
  if(basespeed < MIN_BASE_SPEED)
    basespeed = MIN_BASE_SPEED;

  int batteryread = analogRead(battery); // read battery voltage
  Serial.print("Battery level ");
  Serial.println(batteryread);
  batterycheck();
  Serial.print("Running at ");
  Serial.println(basespeed);

  // Unit test sensors
  #ifdef UNIT_TEST
  delay(30);
  Serial.print("delay\nhigh->");
  Serial.println(radiusMarker.isTriggered(markerHighThreshold + 1));
  Serial.print("low->");
  Serial.println(radiusMarker.isTriggered(markerLowThreshold - 1));
  delay(30);
  Serial.print("delay\nhigh->");
  Serial.println(radiusMarker.isTriggered(markerHighThreshold + 1));
  Serial.print("low->");
  Serial.println(radiusMarker.isTriggered(markerLowThreshold - 1));
  Serial.print("high->");
  Serial.println(radiusMarker.isTriggered(markerHighThreshold + 1));
  Serial.print("low->");
  Serial.println(radiusMarker.isTriggered(markerLowThreshold - 1));
  delay(30);
  Serial.print("delay\nlow->");
  Serial.println(radiusMarker.isTriggered(markerLowThreshold - 1));
  Serial.print("high->");
  Serial.println(radiusMarker.isTriggered(markerHighThreshold + 1));
  Serial.print("low->");
  Serial.println(radiusMarker.isTriggered(markerLowThreshold - 1));
  Serial.print("high->");
  Serial.println(radiusMarker.isTriggered(markerHighThreshold + 1));
  Serial.print("low->");
  Serial.println(radiusMarker.isTriggered(markerLowThreshold - 1));
  delay(30);
  Serial.print("delay\nlow->");
  Serial.println(radiusMarker.isTriggered(markerLowThreshold - 1));
  delay(30);
#endif

  if (fnswvalue > 0) 
  {
    //linefollow(); // line follower routine
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
      replayRecordedPath(fastRunSpeed, basespeed, (int)(basespeed*SLOWDOWN_SPEED_RATIO));
    }
  }
// if (fnswvalue == 1) 
  else
    sensorTest();
}
