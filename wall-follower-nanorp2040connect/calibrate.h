
typedef struct 
{
  int lfrontsensMax;  
  int lfrontsensMin;
  int rfrontsensMax;  
  int rfrontsensMin;
  int rsidesensMax;
  int rsidesensMin;
  int lsidesensMax;
  int lsidesensMin;

} SensorCalibration;

// Simple calibrate methods
// 
// a) Spin the robot slowly
// b) Record the max and mins of each sensor
// c) Scale the value 0-100
// d) Continue to show the readinds, this time scaled
// e) If the button pressed, save the results and exit
void calibrateSensors()
{
    delay(200);

    SensorCalibration cal;
    cal.lfrontsensMax = 0;  cal.lfrontsensMin = 1024;
    cal.rfrontsensMax = 0;  cal.rfrontsensMin = 1024;
    cal.rsidesensMax = 0;   cal.rsidesensMin = 1024;
    cal.lsidesensMax = 0;  cal.lsidesensMin = 1024;

    digitalWrite(rmotorDIR, HIGH); // set right motor forward
    digitalWrite(lmotorDIR, HIGH); // set left motor reverse
    analogWrite(rmotorPWM, 40); // set right motor speed
    analogWrite(lmotorPWM, 40); // set left motor speed

    for(int i = 0; i < 2000; i++)
    {
      photoread();

      cal.lfrontsensMax = std::max(cal.lfrontsensMax, lfrontsens);
      cal.lfrontsensMin = std::min(cal.lfrontsensMin, lfrontsens);
      cal.rfrontsensMax = std::max(cal.rfrontsensMax, rfrontsens);
      cal.rfrontsensMin = std::min(cal.rfrontsensMin, rfrontsens);
      cal.rsidesensMax = std::max(cal.rsidesensMax, rsidesens);
      cal.rsidesensMin = std::min(cal.rsidesensMin, rsidesens);
      cal.lsidesensMax = std::max(cal.lsidesensMax, lsidesens);
      cal.lsidesensMin = std::min(cal.lsidesensMin, lsidesens);

      // Show indicators if above average
      digitalWrite (sensorLED1, rsidesens > (cal.rsidesensMax+cal.rsidesensMin)/2 );  // Right/Red LED
      digitalWrite (sensorLED2, lsidesens > (cal.lsidesensMax+cal.lsidesensMin)/2);   // Left/Green LED
      digitalWrite (indicatorLedBlue, lfrontsens > (cal.lfrontsensMax+cal.lfrontsensMin)/2);  // Centre/Blue LED

      Serial.print(cal.lsidesensMin); Serial.print("/"); Serial.print(cal.lsidesensMax); Serial.print(", "); 
      Serial.print(cal.lfrontsensMin); Serial.print("/"); Serial.print(cal.lfrontsensMax); Serial.print(", "); 
      Serial.print(cal.rfrontsensMin); Serial.print("/"); Serial.print(cal.rfrontsensMax); Serial.print(", "); 
      Serial.print(cal.rfrontsensMin); Serial.print("/"); Serial.print(cal.rfrontsensMax); Serial.println(); 
      delay(2);      
    }

    //Stop
    analogWrite(rmotorPWM, 0); // set right motor speed
    analogWrite(lmotorPWM, 0); // set left motor speed

    bool saved = false;
    while(!saved) 
    {
      photoread();

      // Show indicators if above average
      digitalWrite (sensorLED1, rsidesens > (cal.rsidesensMax+cal.rsidesensMin)/2 );  // Right/Red LED
      digitalWrite (sensorLED2, lsidesens > (cal.lsidesensMax+cal.lsidesensMin)/2);   // Left/Green LED
      digitalWrite (indicatorLedBlue, lfrontsens > (cal.lfrontsensMax+cal.lfrontsensMin)/2);  // Centre/Blue LED

      delay(2);

      if(buttonPressed())
      {
        // TODO - Just use the RADIUS marker for both for now
        markerLowThreshold = (cal.rsidesensMax+cal.rsidesensMin)/2;

        // Flash to confirm
        for(int j=0; j < 5; j++)
        {
          digitalWrite (sensorLED1, HIGH );  // Right/Red LED
          digitalWrite (sensorLED2, HIGH );   // Left/Green LED
          delay(100);
          digitalWrite (sensorLED1, LOW );  // Right/Red LED
          digitalWrite (sensorLED2, LOW );   // Left/Green LED
          delay(100);
        }
        saved = true;
      }
    }
}

void LoadCalibration()
{

}