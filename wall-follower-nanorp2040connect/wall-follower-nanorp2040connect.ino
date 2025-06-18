/* 
  Wall-follower & Line-follower combined version
  for Monty-Pi Arduino 2040 Connect-based UKMARSBOT
*/  

#include <WiFiNINA.h>
#include "defaults.h"
#include "ukmarsbot-pins.h"
#include "wallFollow.h"
#include "lineFollow.h"
#include "calibrate.h"

int basespeed = MIN_BASE_SPEED; //Base speed (constant)

#include "debounce.h"
#include "quadrature.h"

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

//Motor variables
int rightspeed = 0; //Right motor speed
int leftspeed = 0; //Left motor speed

// Encoders
Quadrature_encoder<m1encoder2, m1encoder1> encoder_l;
Quadrature_encoder<m2encoder2, m2encoder1> encoder_r;

// Initialise hardware
void setup() 
{
  Serial.begin(115200);
  Serial1.begin(115200);  
  
  Serial.println("Monty Pi\nLine Follower and Wall Follower");
  Serial1.println("Monty Pi\nLine Follower and Wall Follower");

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

  encoder_l.begin(pull_direction::up, resolution::half);
  encoder_r.begin(pull_direction::up, resolution::half);

}

bool buttonPressed()
{
  switchvoltage = analogRead(fourwayswitch);
  return switchvoltage >= 1000;
}

// Wait for button to be pressed and released
void buttonwait(int period)
{ 
  digitalWrite(indicatorLedBlue, HIGH); // put LED on

  // Wait for button Up
  while (buttonPressed())
  {
    delay(10);
  }

  int flash = 0;
  while (!buttonPressed())
  { 
      delay(10);
      if(++flash % period == 0)
        digitalWrite(indicatorLedBlue, LOW); // put LED off
      else if(flash % period == period/2)
        digitalWrite(indicatorLedBlue, HIGH); // put LED on
  }
  
  digitalWrite(indicatorLedBlue, LOW); // LED off
  
  // Debounce time
  delay(20);

  // Wait for button Up
  while (buttonPressed())
  {
    delay(10);
  }

  digitalWrite(indicatorLedBlue, HIGH); // put LED on

  // Time for finger to be removed...
  delay(250);
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

bool batterycheck()
{ 
  // function to check battery voltage is over 6 volts
  int batteryread = analogRead(battery); // read battery voltage
  if (batteryread > 614) 
    return true; // check if over 6 volts and return if it is
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
    batteryread = analogRead(battery);
  }
  // Now OK
  digitalWrite(sensorLED1, LOW);
  digitalWrite(sensorLED2, LOW);
  return false;
} // end of batterycheck function

void photoread(bool polarity)
{
  // read both line sensors & start/stop sensor
  int lfrontsensAmbient = analogRead(lfront);
  int rfrontsensAmbient = analogRead(rfront);
  int rsidesensAmbient = analogRead(rside);
  int lsidesensAmbient = analogRead(lside);

  digitalWrite(trigger, HIGH);  // Sensor illumination LEDs on

  // Settling time
  delayMicroseconds(ILLUMINATION_ON_TIME_uS);

  // read both line sensors & start/stop sensor
  lfrontsens = analogRead(lfront);
  rfrontsens = analogRead(rfront);
  rsidesens = analogRead(rside);
  lsidesens = analogRead(lside);

  digitalWrite(trigger, LOW);  // Sensor illumination LEDs off

// Remove ambient light level
  if(polarity)
  {
    lfrontsens = lfrontsens > lfrontsensAmbient ? lfrontsens - lfrontsensAmbient : 0;
    rfrontsens = rfrontsens > rfrontsensAmbient ? rfrontsens - rfrontsensAmbient : 0;
    rsidesens = rsidesens > rsidesensAmbient ? rsidesens - rsidesensAmbient : 0;
    lsidesens = lsidesens > lsidesensAmbient ? lsidesens - lsidesensAmbient : 0;
  }
  else
  {
    lfrontsens = lfrontsens < lfrontsensAmbient ? lfrontsens + (1023 - lfrontsensAmbient) : 1023;
    rfrontsens = rfrontsens < rfrontsensAmbient ? rfrontsens + (1023 - rfrontsensAmbient) : 1023;
    rsidesens = rsidesens < rsidesensAmbient ? rsidesens + (1023 - rsidesensAmbient) : 1023;
    lsidesens = lsidesens < lsidesensAmbient ? lsidesens + (1023 - lsidesensAmbient) : 1023;
  }

#ifdef LEDS_FROM_SENSORS
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
#endif

//#define DEBUG_SENS
#ifdef DEBUG_SENS
  DebugPort.print("SENS,");
  DebugPort.print(lsidesens);
  DebugPort.print(",");
  DebugPort.print(lfrontsens);
  DebugPort.print(",");
  DebugPort.print(rfrontsens);
  DebugPort.print(",");
  DebugPort.println(rsidesens);
#endif
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

    // Settling time
    delayMicroseconds(ILLUMINATION_ON_TIME_uS);
    
    // read the value from the sensor:
    int a0val = analogRead(lside);    
    int a1val = analogRead(lfront);    
    int a2val = analogRead(rfront);    
    int a3val = analogRead(rside);    

    digitalWrite(trigger, LOW);  // Sensor illumination LEDs off

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

    // light the right hand sensor LED if white line seen by left sensor
    if (rsidesens > markerLowThreshold)
    {
      digitalWrite (sensorLED1, HIGH);
    }
    else 
    {
      digitalWrite (sensorLED1, LOW);
    }
    // light the left hand sensor LED if white line seen by left sensor
    if (lfrontsens > markerLowThreshold)
    {
      digitalWrite (sensorLED2, HIGH);
    }
    else 
    {
      digitalWrite (sensorLED2, LOW);
    }
    // light the main LED if seen by front sensor
    if (rfrontsens > markerLowThreshold)
    {
      digitalWrite (indicatorLedBlue, HIGH);
    }
    else 
    {
      digitalWrite (indicatorLedBlue, LOW);
    }

    DebugPort.print(a0val);
    DebugPort.print(",");
    DebugPort.print(a0valu);
    DebugPort.print(",");
    DebugPort.print(lsidesens);    
    DebugPort.print(", ");
    DebugPort.print(a1val);
    DebugPort.print(",");
    DebugPort.print(a1valu);
    DebugPort.print(",");
    DebugPort.print(lfrontsens);    
    DebugPort.print(", ");
    DebugPort.print(a2val);
    DebugPort.print(",");
    DebugPort.print(a2valu);
    DebugPort.print(",");
    DebugPort.print(rfrontsens);    
    DebugPort.print(", ");
    DebugPort.print(a3val);
    DebugPort.print(",");
    DebugPort.print(a3valu);
    DebugPort.print(",");
    DebugPort.print(rsidesens);    
    DebugPort.print(", ");
    DebugPort.print(rfrontsens - lfrontsens);
    DebugPort.print(",");
    DebugPort.println(fnswvalue);

    delay(20); 
  }
}

void showMode(int mode)
{
  digitalWrite(LEDR, (mode & 1) ? LOW : HIGH);
  //digitalWrite(LEDG, mode == 1 ? HIGH : LOW); // doesn't seem to work
  digitalWrite(LEDB, mode >= 1 ? HIGH : LOW);
}

void loop() 
{
  int mode = buttonPressed() ? 1 : 0;
  showMode(mode);
  // If pressed for a bit longer, set into mode 2
  if(mode)
  {
    delay(1000);
    if(buttonPressed())
      mode++;
    showMode(mode);
  }

  DebugPort.print("Mode: "); DebugPort.println(mode == 0 ? "Line" : mode == 1 ? "Pursuit" : "Wall");

  // Show mode on sensor LEDs
//  digitalWrite (sensorLED1, altMode & 1 );  // Right/Red LED
//  digitalWrite (sensorLED2, altMode & 2 );  // Left/Green LED

  // Wait for button press and battery voltage ok
  buttonwait((mode + 1) * 25); // wait for function button to be pressed
  while(!batterycheck())
  {
    buttonwait(10); // wait for function button to be pressed
  }

  // Get the base speed from the DIP switches
  delay(30);
  functionswitch(); // read function switch value after button released
  basespeed = fnswvalue * 8;
  if(basespeed < MIN_BASE_SPEED)
    basespeed = MIN_BASE_SPEED;

  int batteryread = analogRead(battery); // read battery voltage
  DebugPort.print("Battery level ");
  DebugPort.println(batteryread);
  //batterycheck();
  DebugPort.print("Running at ");
  DebugPort.println(basespeed);

  // Unit test sensors
  #ifdef UNIT_TEST
  delay(30);
  DebugPort.print("delay\nhigh->");
  DebugPort.println(radiusMarker.isTriggered(markerHighThreshold + 1));
  DebugPort.print("low->");
  DebugPort.println(radiusMarker.isTriggered(markerLowThreshold - 1));
  delay(30);
  DebugPort.print("delay\nhigh->");
  DebugPort.println(radiusMarker.isTriggered(markerHighThreshold + 1));
  DebugPort.print("low->");
  DebugPort.println(radiusMarker.isTriggered(markerLowThreshold - 1));
  DebugPort.print("high->");
  DebugPort.println(radiusMarker.isTriggered(markerHighThreshold + 1));
  DebugPort.print("low->");
  DebugPort.println(radiusMarker.isTriggered(markerLowThreshold - 1));
  delay(30);
  DebugPort.print("delay\nlow->");
  DebugPort.println(radiusMarker.isTriggered(markerLowThreshold - 1));
  DebugPort.print("high->");
  DebugPort.println(radiusMarker.isTriggered(markerHighThreshold + 1));
  DebugPort.print("low->");
  DebugPort.println(radiusMarker.isTriggered(markerLowThreshold - 1));
  DebugPort.print("high->");
  DebugPort.println(radiusMarker.isTriggered(markerHighThreshold + 1));
  DebugPort.print("low->");
  DebugPort.println(radiusMarker.isTriggered(markerLowThreshold - 1));
  delay(30);
  DebugPort.print("delay\nlow->");
  DebugPort.println(radiusMarker.isTriggered(markerLowThreshold - 1));
  delay(30);
#endif

  if (fnswvalue > 1) 
  {
    switch(mode)
    {
      case 0:
        lineFollower(basespeed, false);
        break; 
      case 1:
        lineFollower(basespeed, true);
        break; 
      case 2:
        simpleWallFollower(basespeed);
        break; 
    }
  }
  else if (fnswvalue == 1)
    calibrateSensors(); 
  else
    sensorTest();
}
