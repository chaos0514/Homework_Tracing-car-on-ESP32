#include <Arduino.h>
#include "Pid.h"

const int Motor_channel_PWMA = 14;
const int Motor_channel_PWMB = 15;

int Motor_freq_PWM = 500;
int Motor_resolution_PWM = 10;

void Motor_Init(void)
{
  ledcSetup(Motor_channel_PWMA, Motor_freq_PWM, Motor_resolution_PWM);
  ledcSetup(Motor_channel_PWMB, Motor_freq_PWM, Motor_resolution_PWM);
  ledcAttachPin(PWML, Motor_channel_PWMA);
  ledcAttachPin(PWMR, Motor_channel_PWMB);
}

void IRAM_ATTR Pid_TimeEvent()
{
  velocity_Left = (encoderVal_Left / 780) * 3.1415 * 2.0 * (1000 / PERIOD);
  Output_Left = pidcontrol_L(Target_Left, velocity_Left);
  velocity_Right = (encoderVal_Right / 780) * 3.1415 * 2.0 * (1000 / PERIOD);
  Output_Right = pidcontrol_R(Target_Right, velocity_Right);
  ledcWrite(PWML, 1.0 / 2048.0 * Output_Left + 0.5);
  encoderVal_Left = 0;
  ledcWrite(PWMR, 1.0 / 2048.0 * Output_Right + 0.5);
  encoderVal_Right = 0;
}

int pidcontrol_L(float target, float cTemp_Rightrent) // 1的PID算法
{
  Left_Error = target - cTemp_Rightrent;
  float kp = 6, ti = 100, td = 15, t = PERIOD;
  float q0 = kp * (1 + t / ti + td / t);
  float q1 = -kp * (1 + 2 * td / t);
  float q2 = kp * td / t;
  Temp_Left = Temp_Left + q0 * Left_Error + q1 * Left_Error_I + q2 * Left_Error_II;
  Left_Error_II = Left_Error_I;
  Left_Error_I = Left_Error;
  if (Temp_Left >= 1023)
  {
    Temp_Left = 1023;
  }
  else if (Temp_Left <= -1023)
  {
    Temp_Left = -1023;
  }
  Output_Left = Temp_Left;
  return (int)Output_Left;
}
int pidcontrol_R(float target, float cTemp_Rightrent) // 2的PID算法
{
  Right_Error = target - cTemp_Rightrent;
  float kp = 6, ti = 100, td = 15, t = PERIOD;
  float q0 = kp * (1 + t / ti + td / t);
  float q1 = -kp * (1 + 2 * td / t);
  float q2 = kp * td / t;
  Temp_Right = Temp_Right + q0 * Right_Error + q1 * Right_Error_I + q2 * Right_Error_II;
  Right_Error_II = Right_Error_I;
  Right_Error_I = Right_Error;
  if (Temp_Right >= 1023)
  {
    Temp_Right = 1023;
  }
  else if (Temp_Right <= -1023)
  {
    Temp_Right = -1023;
  }
  Output_Right = Temp_Right;
  return (int)Output_Right;
}
