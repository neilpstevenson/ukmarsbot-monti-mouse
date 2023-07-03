#include <WiFiNINA.h>
#include "defaults.h"
#include "ukmarsbot-pins.h"

int basespeed = MIN_BASE_SPEED; //Base speed (constant)

#include "pid.h"
#include "debounce.h"

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
int startStopCount = 0;

//Motor variables
int rightspeed = 0; //Right motor speed
int leftspeed = 0; //Left motor speed

// Steering
float pidInput = 0.0;
float pidSetpoint = 0.0;
PID steeringPID(PID_Kp, PID_Ki, PID_Kd, &pidInput, &pidSetpoint);

// Markers
#if SENSOR_POLAIRTY_TRUE
// New sensor board
Debounce startFinish(markerLowThreshold, markerHighThreshold, true);
#else
Debounce startFinish(markerLowThreshold, markerHighThreshold, false);
#endif
int startFinishCount = 0;


int i=0; // loop counter

void buttonwait()
{ 
 // waits until tactile button is pressed
 digitalWrite(LED13, HIGH); // put LED on
 switchvoltage = analogRead(fourwayswitch);
 while (switchvoltage < 1000)
 { 
    // while button not pressed
    switchvoltage = analogRead(fourwayswitch);
 }
 digitalWrite(LED13, LOW); // LED off
 // Debounce time
 delay(20);
 while (switchvoltage >= 1000)
 { 
    // while button pressed
    switchvoltage = analogRead(fourwayswitch);
 }
 digitalWrite(LED13, HIGH); // put LED on
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
  while (1) // endless loop
  {
    digitalWrite(LED13, HIGH); // switch on LED
    delay (100); // wait 1/10 second
    digitalWrite(LED13, LOW); // switch off LED
    delay (500); // wait 1/2 second
  }
} // end of batterycheck function

void photoread()
{
 // read both line sensors & start/stop sensor
 lfrontsens = analogRead(lfront);
 rfrontsens = analogRead(rfront);
 rsidesens = analogRead(rside);
 lsidesens = analogRead(lside);

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

#ifdef DEBUG_SENS
 Serial.print(lfrontsens);
 Serial.print("/");
 Serial.print(rfrontsens);
 Serial.print(" ss=");
 Serial.println(rsidesens);
#endif
 
 // Start/Stop check
 if(startFinish.isTriggered(rsidesens))
 {
    startStopCount++;
    Serial.println("triggered");
 }
}

void linefollow() 
{
  unsigned long int count = 0;
  int endStopLineCount = START_STOP_COUNT_DEFAULT;

  rightspeed = basespeed;
  leftspeed = basespeed;
  digitalWrite(rmotorDIR, HIGH); // set right motor forward
  digitalWrite(lmotorDIR, LOW); // set left motor forward
  
  digitalWrite(trigger, HIGH);  // Sensor illumination LEDs on


  // Wait for marker to go off, as alternative to dragster start 
  // tirgger
  photoread();
  Serial.print("lsidesens="); Serial.println(lsidesens);
#if SENSOR_POLAIRTY_TRUE
  while(lsidesens > markerHighThreshold)
#else
  while(lsidesens < markerHighThreshold)
#endif
  {
    delay(10);
    photoread();
    Serial.print("lsidesens="); Serial.println(lsidesens);

    // Assume a Dragster course
    endStopLineCount = START_STOP_COUNT_DRAGSTER;
    steeringPID.set_tunings(PID_Kp_DRAGSTER, PID_Ki_DRAGSTER, PID_Kd_DRAGSTER);
  }

  startStopCount = 0;
  unsigned long int startTime = millis();
  analogWrite(rmotorPWM, rightspeed); // set right motor speed
  analogWrite(lmotorPWM, leftspeed); // set left motor speed

  while(startStopCount < endStopLineCount)
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
    if(turn > 0)
    {
      //rightspeed = basespeed + (turn*2/3);
      rightspeed = basespeed + turn;
      //rightspeed = basespeed;
      leftspeed = basespeed - turn;
      //leftspeed = basespeed - (turn*2/3);
    }
    else
    {
      rightspeed = basespeed + turn;
      //rightspeed = basespeed + (turn*2/3);
      //leftspeed = basespeed;
      leftspeed = basespeed - turn;
      //leftspeed = basespeed - (turn*2/3);
    }
    analogWrite(rmotorPWM, rightspeed >= 0 ? rightspeed : 0); // set right motor speed
    analogWrite(lmotorPWM, leftspeed >= 0 ? leftspeed : 0); // set left motor speed
    
    Serial.print(leftspeed);
    Serial.print(",");
    Serial.println(rightspeed);
    
    count++;
    delay(3);
  }

  unsigned long int endTime = millis();

  digitalWrite(LED13, LOW); // LED off
  
  // Slowdown sequence
  int SLOWDOWN_TIME = 30000/basespeed/SLOWDOWN_SPEED_RATIO;
  for(int i = 0; i < SLOWDOWN_TIME; i++)
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
    rightspeed = basespeed + turn; //
    leftspeed = basespeed - turn; //
    analogWrite(rmotorPWM, rightspeed/SLOWDOWN_SPEED_RATIO); // set right motor speed
    analogWrite(lmotorPWM, leftspeed/SLOWDOWN_SPEED_RATIO); // set left motor speed
    
    delay(3);
  }
  
  stopmotors();

  Serial.print("Time mS: "); Serial.println(endTime-startTime);
  Serial.print("Loop: "); Serial.print(count);
  Serial.print(" ("); Serial.print(count/((endTime-startTime)/1000.0)); Serial.println("/sec)"); 
  
  // pause to see led off
  delay(500);
}

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
  pinMode(LED13, OUTPUT);
  pinMode(m1encoder1, INPUT);
  pinMode(m1encoder2, INPUT);
  pinMode(m2encoder1, INPUT);
  pinMode(m2encoder2, INPUT);
  
  stopmotors(); // switch off the motors
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

    Serial.print(a0val);
    Serial.print(",");
    Serial.print(a0valu);
    Serial.print(",");
    Serial.print(a1val);
    Serial.print(",");
    Serial.print(a1valu);
    Serial.print(",");
    Serial.print(a2val);
    Serial.print(",");
    Serial.print(a2valu);
    Serial.print(",");
    Serial.print(a3val);
    Serial.print(",");
    Serial.print(a3valu);
    Serial.print(",");
    Serial.println(fnswvalue);

    digitalWrite(trigger, LOW);  // Sensor illumination LEDs off
    delay(20); 
  }
}

void loop() 
{
  // Get the base speed from the DIP switches
  buttonwait(); // wait for function button to be pressed
  functionswitch(); // read function switch value after button released
  basespeed = fnswvalue * 17;
  if(basespeed < MIN_BASE_SPEED)
    basespeed = MIN_BASE_SPEED;

  int batteryread = analogRead(battery); // read battery voltage
  Serial.print("Battery level ");
  Serial.println(batteryread);
  Serial.print("Running at ");
  Serial.println(basespeed);
  
  if (fnswvalue > 0) 
    linefollow(); // line follower routine
// if (fnswvalue == 1) 
  else
    sensorTest();


}
