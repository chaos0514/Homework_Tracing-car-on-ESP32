#ifndef TIMER_H
#define TIMER_H

#include <Arduino.h>

#define PWML 22 
#define PWMR 23 

extern volatile float Target_Left;
extern volatile float Target_Right;
extern volatile float encoderVal_Left;
extern volatile float encoderVal_Right;
extern volatile float velocity_Left;
extern volatile float velocity_Right;
extern volatile float Temp_Left;
extern volatile float Temp_Right;
extern volatile float Left_Error;
extern volatile float Left_Error_I;
extern volatile float Left_Error_II;
extern volatile int Output_Left;
extern volatile float Right_Error;
extern volatile float Right_Error_I;
extern volatile float Right_Error_II;
extern volatile int Output_Right;

#define PWML 22 // 给电机1PWM输出调速
#define PWMR 23 // 给电机2PWM输出调速
#define PERIOD 20

void Motor_Init(void);
int pidcontrol_R(float target, float cTemp_Rightrent);
int pidcontrol_L(float target, float cTemp_Rightrent);
void IRAM_ATTR Pid_TimeEvent();




#endif