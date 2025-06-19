/* 
  Wall-follower & Line-follower combined version
  for Monty-Pi Arduino 2040 Connect-based UKMARSBOT
*/  

#include <WiFiNINA.h>
#include "defaults.h"
//#include "ukmarsbot-pins.h"

// These are taken from ukmars mazerunner-core
#include "config.h"
#include "adc.h"
#include "battery.h"
//#include "cli.h"
#include "encoders.h"
//#include "maze.h"
#include "motion.h"
#include "motors.h"
//#include "mouse.h"
//#include "reporting.h"
#include "sensors.h"
#include "switches.h"
#include "systick.h"

#include "wallFollow.h"
#include "pathRecorder.h"
#include "lineFollow.h"
#include "calibrate.h"

int basespeed = MIN_BASE_SPEED; //Base speed (constant)

#include "debounce.h"
#include "quadrature.h"

/******************************************************************************/

// Global objects are all defined here. The declarations are in
// the corresponding header files
Systick systick;                          // the main system control loop
AnalogueConverter adc;                    // controls all analogue conversions
Battery battery(BATTERY_ADC_CHANNEL);     // monitors battery voltage
Switches switches(SWITCHES_ADC_CHANNEL);  // monitors the button and switches
Encoders encoders;                        // tracks the wheel encoder counts
Sensors sensors;                          // make sensor alues from adc vdata
Motion motion;                            // high level motion operations
Motors motors;                            // low level control for drive motors
Profile forward;                          // speed profiles for forward motion
Profile rotation;                         // speed profiles for rotary motion
//Maze maze PERSISTENT;                     // holds maze map (even after a reset)
//Mouse mouse;                              // all the main robot logic is here
//CommandLineInterface cli;                 // user interaction on the serial port
//Reporter reporter;                        // formatted reporting of robot state
//Indicators indicators;                    // More complex indicators/display

//Inputs
/*
int rfrontsens = 0; //Right front sensor value
int lfrontsens = 0; //Left front sensor value
int rsidesens = 0; //Right side sensor value
int lsidesens = 0; //Left side sensor value
*/
int sensdiff = 0; //Difference between front sensors
/*int batteryvolts = 0; // battery voltage reading
int batterycalc = 0; // working field
int switchvoltage = 0; // analogue value coming back from reading function or 4 way switch
int fnswvalue = 0; // value (in range 0 to 16) of 4 way function switch
int posn = 0; // if on line or off it and which side
*/
//Motor variables
int rightspeed = 0; //Right motor speed
int leftspeed = 0; //Left motor speed
/*
// Encoders
Quadrature_encoder<m1encoder2, m1encoder1> encoder_l;
Quadrature_encoder<m2encoder2, m2encoder1> encoder_r;
*/

// Initialise hardware
void setup() 
{
  Serial.begin(BAUDRATE); // Need to do this always, else it prevents the USB programmer/bootloader being available
#ifdef USE_USB_SERIAL_PORT 
  delay(1000);      // Allow any USB SerialPort to be established
  //redirectPrintf(); // send printf output to SerialPort (uses 20 bytes RAM)
#else
  // Also the serial port
  SerialPort.begin(BAUDRATE);
#endif

  pinMode(LED_LEFT_IO, OUTPUT);
  digitalWrite(LED_LEFT_IO, 0);
  pinMode(LED_RIGHT_IO, OUTPUT);
  digitalWrite(LED_RIGHT_IO, 0);
  //pinMode(SWITCH_GO_PIN, INPUT_PULLUP);
  //pinMode(SWITCH_SELECT_PIN, INPUT_PULLUP);
  adc.begin();
  motors.begin();
  encoders.begin();
  /// do not begin systick until the hardware is setup
  systick.begin();

  /// leave the emitters off unless we are actually using the sensors
  /// less power, less risk
  sensors.disable();

  SerialPort.println();
  SerialPort.println("Monty Pi Line & Wall Follower");
}

// Wait for button to be pressed and released
void buttonwait(int period)
{ 
  digitalWrite(indicatorLedBlue, HIGH); // put LED on

  // Wait for button Up
  while (switches.button_pressed())
  {
    delay(10);
  }

  int flash = 0;
  while (!switches.button_pressed())
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
  while (switches.button_pressed())
  {
    delay(10);
  }

  digitalWrite(indicatorLedBlue, HIGH); // put LED on

  // Time for finger to be removed...
  delay(250);
}

bool batterycheck()
{ 
  // function to check battery voltage is over 6 volts
  int batteryread = battery.voltage();
  if (batteryread > 6.0) 
    return true; // check if over 6 volts and return if it is
  motors.stop();
  motors.disable_controllers();
  while (batteryread < 6.0)
  {
    // Flash all LEDs
    digitalWrite(indicatorLedBlue, HIGH);
    digitalWrite(LED_LEFT_IO, LOW);
    digitalWrite(LED_RIGHT_IO, LOW);
    delay (100); // wait 1/10 second
    digitalWrite(indicatorLedBlue, LOW);
    digitalWrite(LED_LEFT_IO, HIGH);
    digitalWrite(LED_RIGHT_IO, HIGH);
    delay (100); // wait 1/2 second
    batteryread = battery.voltage();
  }
  // Now OK
  digitalWrite(LED_LEFT_IO, LOW);
  digitalWrite(LED_RIGHT_IO, LOW);
  return false;
} // end of batterycheck function

/*  
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
    digitalWrite (LED_LEFT_IO, HIGH);
  }
  else 
  {
    digitalWrite (LED_LEFT_IO, LOW);
  }
  // light the left hand sensor LED if white line seen by left sensor
  if (lfrontsens < sensorthreshold)
  {
    digitalWrite (LED_RIGHT_IO, HIGH);
  }
  else 
  {
    digitalWrite (LED_RIGHT_IO, LOW);
  }
#endif

//#define DEBUG_SENS
#ifdef DEBUG_SENS
  SerialPort.print("SENS,");
  SerialPort.print(lsidesens);
  SerialPort.print(",");
  SerialPort.print(lfrontsens);
  SerialPort.print(",");
  SerialPort.print(rfrontsens);
  SerialPort.print(",");
  SerialPort.println(rsidesens);
#endif
}
*/


void sensorTest()
{
  /*
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
      digitalWrite (LED_LEFT_IO, HIGH);
    }
    else 
    {
      digitalWrite (LED_LEFT_IO, LOW);
    }
    // light the left hand sensor LED if white line seen by left sensor
    if (lfrontsens > markerLowThreshold)
    {
      digitalWrite (LED_RIGHT_IO, HIGH);
    }
    else 
    {
      digitalWrite (LED_RIGHT_IO, LOW);
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

    SerialPort.print(a0val);
    SerialPort.print(",");
    SerialPort.print(a0valu);
    SerialPort.print(",");
    SerialPort.print(lsidesens);    
    SerialPort.print(", ");
    SerialPort.print(a1val);
    SerialPort.print(",");
    SerialPort.print(a1valu);
    SerialPort.print(",");
    SerialPort.print(lfrontsens);    
    SerialPort.print(", ");
    SerialPort.print(a2val);
    SerialPort.print(",");
    SerialPort.print(a2valu);
    SerialPort.print(",");
    SerialPort.print(rfrontsens);    
    SerialPort.print(", ");
    SerialPort.print(a3val);
    SerialPort.print(",");
    SerialPort.print(a3valu);
    SerialPort.print(",");
    SerialPort.print(rsidesens);    
    SerialPort.print(", ");
    SerialPort.print(rfrontsens - lfrontsens);
    SerialPort.print(",");
    SerialPort.println(fnswvalue);

    delay(20); 
  }
  */
}

void showMode(int mode)
{
  digitalWrite(LEDR, (mode & 1) ? LOW : HIGH);
  //digitalWrite(LEDG, mode == 1 ? HIGH : LOW); // doesn't seem to work
  digitalWrite(LEDB, mode >= 1 ? HIGH : LOW);
}

void loop() 
{
  int mode = switches.button_pressed() ? 1 : 0;
  showMode(mode);
  // If pressed for a bit longer, set into mode 2
  if(mode)
  {
    delay(1000);
    if(switches.button_pressed())
      mode++;
    showMode(mode);
  }

  SerialPort.print("Mode: "); SerialPort.println(mode == 0 ? "Line" : mode == 1 ? "Pursuit" : "Wall");

  // Show mode on sensor LEDs
//  digitalWrite (LED_LEFT_IO, altMode & 1 );  // Right/Red LED
//  digitalWrite (LED_RIGHT_IO, altMode & 2 );  // Left/Green LED

  // Wait for button press and battery voltage ok
  buttonwait((mode + 1) * 25); // wait for function button to be pressed
  while(!batterycheck())
  {
    buttonwait(10); // wait for function button to be pressed
  }

  // Get the base speed from the DIP switches
  delay(30);
  int fnswvalue = switches.read(); // read function switch value after button released
  basespeed = fnswvalue * LINE_FOLLOWER_SPEED_PER_SWITCH; // mm/s
  if(basespeed < MIN_BASE_SPEED)
    basespeed = MIN_BASE_SPEED;

  float batteryread = battery.voltage();
  SerialPort.print("Battery level ");
  SerialPort.println(batteryread);
  //batterycheck();
  SerialPort.print("Running at ");
  SerialPort.println(basespeed);

  // Unit test sensors
  #ifdef UNIT_TEST
  delay(30);
  SerialPort.print("delay\nhigh->");
  SerialPort.println(radiusMarker.isTriggered(markerHighThreshold + 1));
  SerialPort.print("low->");
  SerialPort.println(radiusMarker.isTriggered(markerLowThreshold - 1));
  delay(30);
  SerialPort.print("delay\nhigh->");
  SerialPort.println(radiusMarker.isTriggered(markerHighThreshold + 1));
  SerialPort.print("low->");
  SerialPort.println(radiusMarker.isTriggered(markerLowThreshold - 1));
  SerialPort.print("high->");
  SerialPort.println(radiusMarker.isTriggered(markerHighThreshold + 1));
  SerialPort.print("low->");
  SerialPort.println(radiusMarker.isTriggered(markerLowThreshold - 1));
  delay(30);
  SerialPort.print("delay\nlow->");
  SerialPort.println(radiusMarker.isTriggered(markerLowThreshold - 1));
  SerialPort.print("high->");
  SerialPort.println(radiusMarker.isTriggered(markerHighThreshold + 1));
  SerialPort.print("low->");
  SerialPort.println(radiusMarker.isTriggered(markerLowThreshold - 1));
  SerialPort.print("high->");
  SerialPort.println(radiusMarker.isTriggered(markerHighThreshold + 1));
  SerialPort.print("low->");
  SerialPort.println(radiusMarker.isTriggered(markerLowThreshold - 1));
  delay(30);
  SerialPort.print("delay\nlow->");
  SerialPort.println(radiusMarker.isTriggered(markerLowThreshold - 1));
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
