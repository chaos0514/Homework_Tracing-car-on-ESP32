#ifndef _TRACE_H
#define _TRACE_H

#include <Arduino.h>

#define TRACE_MID 21
#define TRACE_LEFT_1 25
#define TRACE_LEFT_2 26
#define TRACE_LEFT_3 27
#define TRACE_RIGHT_1 13
#define TRACE_RIGHT_2 16
#define TRACE_RIGHT_3 17
#define velocity_max 20

extern volatile float Target_Left;
extern volatile float Target_Right;
extern volatile float Target_Left_Temp;
extern volatile float Target_Right_Temp;

void IRAM_ATTR Trace_TimeEvent();
void Trace_Init();
void Target_Left_Cal();
void Target_Right_Cal();

#endif