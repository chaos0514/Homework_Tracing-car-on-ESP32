#include "Trace.h"

void Trace_Init(){
    pinMode(TRACE_MID,INPUT);
    pinMode(TRACE_LEFT_1,INPUT);
    pinMode(TRACE_LEFT_2,INPUT);
    pinMode(TRACE_LEFT_3,INPUT);
    pinMode(TRACE_RIGHT_1,INPUT);
    pinMode(TRACE_RIGHT_2,INPUT);
    pinMode(TRACE_RIGHT_3,INPUT);
}
void IRAM_ATTR Trace_TimeEvent()
{
    Target_Left_Cal();
    Target_Right_Cal();
}
void Target_Left_Cal()
{
    Target_Left_Temp = Target_Left;
    Target_Right_Temp = Target_Right;
    if (digitalRead(TRACE_MID) == LOW)
    {
        Target_Left = velocity_max;
        Target_Right = velocity_max;
    }
    if (digitalRead(TRACE_LEFT_1) == LOW) // 轻微轻微右转
    {
        Target_Left = 0.9 * velocity_max;
        Target_Right = 0.8 * velocity_max;
    }
    if (digitalRead(TRACE_LEFT_2) == LOW) // 右转
    {
        Target_Left = 0.9 * velocity_max;
        Target_Right = 0.6 * velocity_max;
    }
    if (digitalRead(TRACE_LEFT_3) == LOW) // 90°右转
    {
        Target_Left = 1 * velocity_max;
        Target_Right = -1 * velocity_max;
    }
    Target_Left = Target_Left * 0.6 + Target_Left_Temp * 0.8;
    Target_Right = Target_Right * 0.6 + Target_Left_Temp * 0.8;
}
void Target_Right_Cal()
{
    Target_Left_Temp = Target_Left;
    Target_Right_Temp = Target_Right;
    if (digitalRead(TRACE_MID) == LOW)
    {
        Target_Left = velocity_max;
        Target_Right = velocity_max;
    }
    if (digitalRead(TRACE_RIGHT_1) == LOW) // 轻微轻微左转
    {
        Target_Left = 0.8 * velocity_max;
        Target_Right = 0.9 * velocity_max;
    }
    if (digitalRead(TRACE_RIGHT_2) == LOW) // 左转
    {
        Target_Left = 0.6 * velocity_max;
        Target_Right = 0.9 * velocity_max;
    }
    if (digitalRead(TRACE_RIGHT_3) == LOW) // 90°左转
    {
        Target_Left = -1 * velocity_max;
        Target_Right = 1 * velocity_max;
    }
    Target_Left = Target_Left * 0.6 + Target_Left_Temp * 0.8;
    Target_Right = Target_Right * 0.6 + Target_Left_Temp * 0.8;
}