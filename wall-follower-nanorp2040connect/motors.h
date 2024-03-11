//#include "hardware.h"
//#include "config.h"

class Motor
{
  private:
    int pinDir;
    int pinPwm;
    
  public:
    Motor(int pinDir, int pinPwm)
      : pinDir(pinDir), pinPwm(pinPwm)
    {
    //  pinMode(pinDir, OUTPUT);
    //  pinMode(pinPwm, OUTPUT);
      // Set initial state
    //  digitalWrite(pinDir, LOW);
    //  analogWrite(pinPwm, 0);
    }

    // Set power, -255 to +255
    void setPower(int power)
    {
      // Break mode
      if(power >= 0)
      {
        if(power > 255)
          power = 255;
        digitalWrite(pinDir, HIGH);
        analogWrite(pinPwm, power);
        //Serial.print("M: ");Serial.print(pinDir);Serial.print(": H, ");Serial.print(pinPwm);Serial.print(": ");Serial.println(power);
      }
      else
      {
        if(power < -255)
          power = -255;
        digitalWrite(pinDir, LOW);
        analogWrite(pinPwm, -power);
        //Serial.print("M: ");Serial.print(pinDir);Serial.print(": L, ");Serial.print(pinPwm);Serial.print(": ");Serial.println(power);
      }
    } 

    void stop(bool breakMode = false)
    {
      analogWrite(pinPwm, 0);
      //Serial.print("M: ");Serial.print(pinPwm);Serial.print(": ");Serial.println(0);
    }      
};

class Motors
{
  private:
    Motor left;
    Motor right;
    
  public:
    Motors(int leftDir = lmotorDIR, int leftPwm = lmotorPWM, int rightDir = rmotorDIR, int rightPwm = rmotorPWM)
      : left(leftDir, leftPwm), right(rightDir, rightPwm)
    {
    }

    // Move forward/reverse -255 to 255
    void forwardPower(int power)
    {
      if(power >= 0)
      {
        left.setPower(-(int)(power * motor_compensation_left));
        right.setPower((int)(power * motor_compensation_right));
      }
      else
      {
        left.setPower(-(int)(power * motor_compensation_left));
        right.setPower((int)(power * motor_compensation_right));
      }
    }

    // Move forward/reverse -255 to 255
    void turn(int power, int turn)
    {
      left.setPower(-((int)(power * motor_compensation_left) - turn));
      right.setPower(((int)(power * motor_compensation_right) + turn));
    }

    void stop(bool breakMode = false)
    {
      left.stop(breakMode);
      right.stop(breakMode);
    }
};
