//#include "hardware.h"
//#include "config.h"

class Motor
{
  private:
    int pinDir;
    // we use the mbed call rather than the Arduino interfaces to access the PWM channels as it gives us more 
    // control of PWM frequency etc. and also a much faster interface
    mbed::PwmOut motorPWM;
    
  public:
    Motor(int pinDir, int pinPwm)
      : pinDir(pinDir), 
        motorPWM(PinName(pinPwm))
    {}

    void begin()
    {
      pinMode(pinDir, OUTPUT);
      motorPWM.period(1.0/20000); // 20kHz pwm
      // Initial state
      digitalWrite(pinDir, LOW);
      motorPWM.write(0);
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
        motorPWM.write(power/255.0);
        DebugPort.print("M: ");DebugPort.print(pinDir);DebugPort.print(": H, ");//DebugPort.print(pinPwm);
        DebugPort.print(": ");DebugPort.println(power);
      }
      else
      {
        if(power < -255)
          power = -255;
        digitalWrite(pinDir, LOW);
        motorPWM.write(-power/255.0);
        DebugPort.print("M: ");DebugPort.print(pinDir);DebugPort.print(": L, ");//DebugPort.print(pinPwm);
        DebugPort.print(": ");DebugPort.println(power);
      }
    } 

    void stop(bool breakMode = false)
    {
      motorPWM.write(0);
      //DebugPort.print("M: ");DebugPort.print(pinPwm);DebugPort.print(": ");DebugPort.println(0);
    }      
};

class Motors
{
  public:
    Motor left;
    Motor right;
    
  public:
    Motors(int leftDir = lmotorDIR, int leftPwm = lmotorPWM_GPIO, int rightDir = rmotorDIR, int rightPwm = rmotorPWM_GPIO)
      : left(leftDir, leftPwm), right(rightDir, rightPwm)
    {
    }

    void begin()
    {
      DebugPort.println("Motors::begin()");
      left.begin();
      right.begin();
    }

    // Move forward/reverse -255 to 255
    void forwardPower(int power)
    {
      DebugPort.print("forwardPower: ");DebugPort.println(power);
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

    // Control individual motors
    void setMotorPowers(int leftPower, int rightPower)
    {
      //DebugPort.print("setMotorPower: ");DebugPort.print(left);DebugPort.print(", ");DebugPort.println(right);
      left.setPower(-(int)(leftPower * motor_compensation_left));
      right.setPower((int)(rightPower * motor_compensation_right));
    }

    // Move forward/reverse -255 to 255
    void turn(int power, int turn)
    {
      DebugPort.print("turn: ");DebugPort.print(power);DebugPort.print(", ");DebugPort.println(turn);
      left.setPower(-((int)(power * motor_compensation_left) - turn));
      right.setPower(((int)(power * motor_compensation_right) + turn));
    }

    void stop(bool breakMode = false)
    {
      left.stop(breakMode);
      right.stop(breakMode);
    }
};

extern Motors motors;
