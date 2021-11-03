#include<TimerOne.h>
#include <PID_v2.h>
#include <MPU6050.h>
#include<Wire.h>
#include <I2Cdev.h>

#define IN3 3
#define IN4 5 
#define IN2 6
#define IN1 9 

MPU6050 mpu;
int16_t accY, accZ;
float accAngle;

const float Kp = 8;
const float Ki = 0.5;
const float Kd = 0;
float input, output, Setpoint;
PID_v2 myPID(Kp, Ki, Kd, PID::Direct);
//||||||||||||||||||||||||||||||||||||||||||
void setup()
{
  mpu.initialize();
  Serial.begin(9600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Setpoint = 0;
  myPID.Start(input, 0, Setpoint);
  myPID.SetSampleTime(2);
  myPID.SetOutputLimits(-255, 255);
}
//||||||||||||||||||||||||||||||||||||||||||
void loop()
{
  float pre_time = millis();
  MPUread();
  calPID();
  balance(output);
  Serial.println(millis() - pre_time); // sample rate
}
//||||||||||||||||||||||||||||||||||||||||||
void MPUread()
{
  accZ = mpu.getAccelerationZ();
  accY = mpu.getAccelerationY();
  accAngle = atan2(accY, accZ) * RAD_TO_DEG;
  //Serial.println(accAngle);
}
//||||||||||||||||||||||||||||||||||||||||||
void calPID()
{
  input = (int)accAngle;
  output = myPID.Run(input);
}
//||||||||||||||||||||||||||||||||||||||||||
void balance(float speed)
{
  if (speed >= 0)
  {
    analogWrite(IN1, speed);
    analogWrite(IN2, 0);
    analogWrite(IN4, speed);
    analogWrite(IN3, 0);
  }
  if (speed < 0)
  {
    speed = - speed;
    analogWrite(IN2, speed);
    analogWrite(IN1, 0);
    analogWrite(IN3, speed);
    analogWrite(IN4, 0);
  }
}
