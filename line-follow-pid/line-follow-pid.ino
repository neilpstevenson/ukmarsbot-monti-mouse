#include "defaults.h"
#include "ukmarsbot-pins.h"
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
int basespeed = BASE_SPEED; //Base speed (constant)
//int turn = 25;  // Max turn
int rightspeed = 0; //Right motor speed
int leftspeed = 0; //Left motor speed

// Steering
float pidInput = 0.0;
float pidSetpoint = 0.0;
PID steeringPID(PID_Kp, PID_Ki, PID_Kd, &pidInput, &pidSetpoint);

// Markers
Debounce startFinish(markerLowThreshold, markerHighThreshold, false);
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
 if (switchvoltage > 87) fnswvalue = fnswvalue - 1;
 if (switchvoltage > 171) fnswvalue = fnswvalue - 1;
 if (switchvoltage > 242) fnswvalue = fnswvalue - 1;
 if (switchvoltage > 310) fnswvalue = fnswvalue - 1;
 if (switchvoltage > 369) fnswvalue = fnswvalue - 1;
 if (switchvoltage > 411) fnswvalue = fnswvalue - 1;
 if (switchvoltage > 449) fnswvalue = fnswvalue - 1;
 if (switchvoltage > 492) fnswvalue = fnswvalue - 1;
 if (switchvoltage > 532) fnswvalue = fnswvalue - 1;
 if (switchvoltage > 556) fnswvalue = fnswvalue - 1;
 if (switchvoltage > 579) fnswvalue = fnswvalue - 1;
 if (switchvoltage > 601) fnswvalue = fnswvalue - 1; 
 if (switchvoltage > 622) fnswvalue = fnswvalue - 1;
 if (switchvoltage > 638) fnswvalue = fnswvalue - 1; 
 if (switchvoltage < 654) fnswvalue = fnswvalue - 1;
 if (switchvoltage > 654) fnswvalue = 0;
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
 // read both line sensors
 lfrontsens = analogRead(lfront);
 rfrontsens = analogRead(rfront);
 
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

 // Start/Stop check
 if(startFinish.isTriggered(analogRead(rside)))
 {
    startStopCount++;
 }
}

void linefollow() 
{
  startStopCount = 0;
  rightspeed = basespeed;
  leftspeed = basespeed;
  digitalWrite(rmotorDIR, HIGH); // set right motor forward
  digitalWrite(lmotorDIR, LOW); // set left motor forward
  analogWrite(rmotorPWM, rightspeed); // set right motor speed
  analogWrite(lmotorPWM, leftspeed); // set left motor speed
  
  digitalWrite(trigger, HIGH);  // Sensor illumination LEDs

  while(startStopCount < START_STOP_COUNT)
  {
    photoread();
    sensdiff = lfrontsens - rfrontsens;

    // Push through PID controller
    pidInput = sensdiff;
    int turn = (int)steeringPID.compute();

    // Set the motors to the default speed +/- turn
    rightspeed = basespeed + turn; //
    leftspeed = basespeed - turn; //
    analogWrite(rmotorPWM, rightspeed); // set right motor speed
    analogWrite(lmotorPWM, leftspeed); // set left motor speed

   // delay(3);
  }
  
  digitalWrite(LED13, LOW); // LED off
  
  // Slowdown sequence
  for(int i = 0; i < SLOWDOWN_TIME; i++)
  {
    photoread();
    sensdiff = lfrontsens - rfrontsens;

    // Push through PID controller
    pidInput = sensdiff;
    int turn = (int)steeringPID.compute();

    // Set the motors to the default speed +/- turn
    rightspeed = basespeed + turn; //
    leftspeed = basespeed - turn; //
    analogWrite(rmotorPWM, rightspeed/SLOWDOWN_SPEED_RATIO); // set right motor speed
    analogWrite(lmotorPWM, leftspeed/SLOWDOWN_SPEED_RATIO); // set left motor speed
  }
  
  stopmotors();
  
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

void loop() 
{
 buttonwait(); // wait for function button to be pressed
 functionswitch(); // read function switch value after button released
// if (fnswvalue == 0) 
  linefollow(); // line follower routine
// if (fnswvalue == 1) phototest();
}
