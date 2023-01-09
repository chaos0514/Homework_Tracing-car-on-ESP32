#include <Arduino.h>
#include "../lib/PID/Pid.h"
#include "../lib/Encoder/Encoder.h"
#include "../lib/Trace/Trace.h"
#include "../lib/Servo/myServo.h"

hw_timer_t *Pid_tim = NULL;
hw_timer_t *Motor_tim = NULL;
hw_timer_t *Trace_tim = NULL;

#define ENCODER_A_LEFT 32
#define ENCODER_B_LEFT 33
#define ENCODER_A_RIGHT 34
#define ENCODER_B_RIGHT 35
#define PWML 22
#define PWMR 23

volatile float Target_Left;
volatile float Target_Right;
volatile float Target_Left_Temp;
volatile float Target_Right_Temp;
volatile float encoderVal_Left;
volatile float encoderVal_Right;
volatile float velocity_Left;
volatile float velocity_Right;
volatile float Temp_Left;
volatile float Temp_Right;
volatile float Left_Error;
volatile float Left_Error_I;
volatile float Left_Error_II;
volatile int Output_Left;
volatile float Right_Error;
volatile float Right_Error_I;
volatile float Right_Error_II;
volatile int Output_Right;

void setup()
{
  Serial.begin(9600);
  Motor_Init(); // 电机初始化
  // ServoInit(); //机械臂初始化
  Trace_Init(); // 红外初始化
/********速度环**********/
  Pid_tim = timerBegin(1, 80, true); // 1号时钟每20ms执行一次pid算法并且驱动电机
  timerAttachInterrupt(Pid_tim, &Pid_TimeEvent, true);
  timerAlarmWrite(Pid_tim, 20000, true);
  timerAlarmEnable(Pid_tim);
/********位置环**********/
  Trace_tim = timerBegin(0, 80, true); // 0号时钟每20ms读取循迹路线
  timerAttachInterrupt(Trace_tim, &Trace_TimeEvent, true);
  timerAlarmWrite(Trace_tim, 20000, true);
  timerAlarmEnable(Trace_tim);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_LEFT), getEncoder_L, CHANGE);  // 通过左电机A相的电平变化触发外部中断，得到脉冲
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_RIGHT), getEncoder_R, CHANGE); // 通过右电机A相的电平变化触发外部中断，得到脉冲
}

void loop()
{
  // Serial.println("velocity_Left:");
  //  Serial.println(velocity_Left);
  // Serial.println("\t");
  // Serial.println("velocity_Right:");
  // Serial.println(velocity_Right);
  // Serial.println("\t\n");

  // Serial.println("encoderVal_Right:");
  // Serial.println(encoderVal_Right);
  // Serial.println("\t");
  // Serial.println("encoderVal_Left:");
  // Serial.println(encoderVal_Left);
  // Serial.println("\t\n");

  // Serial.println("ROUTPUT:");
  // Serial.println(Output_Right);
  // Serial.println("\t");
  // Serial.println("Output_Left:");
  // Serial.println(Output_Left);
  // Serial.println("\r\n");
  delay(10);
}