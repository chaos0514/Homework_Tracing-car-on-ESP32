#ifndef _ENCODER_H
#define _ENCODER_H
#include <Arduino.h>
#define ENCODER_A_LEFT 2  // 电机1的A相输出口接2号中断口
#define ENCODER_B_LEFT 4  // 电机1的B相输出口接4号数字口
#define ENCODER_A_RIGHT 3 // 电机2的A相输出口接3号中断口
#define ENCODER_B_RIGHT 5 // 电机2的B相输出口接5号数字口
#define PWML 10           // 给电机1PWM输出调速
#define INL1 12           // 电机1方向：11，12
#define INL2 11
#define PWMR 9 // 给电机2PWM输出调速
#define INR1 7 // 电机2方向：7、8
#define INR2 8
#define PERIOD 20
void getEncoder_L(void);
void getEncoder_R(void);
#endif