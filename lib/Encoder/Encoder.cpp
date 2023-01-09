#include "Encoder.h"

extern volatile float encoderVal_Left;
extern volatile float encoderVal_Right;

/*
请注意！！！
读取编码器的脉冲一定要考虑电机的转向
转向不同时，编码器digitalRead()所判断的条件相反
否则电机会失控
*/
void getEncoder_L(void)
{
    if (digitalRead(ENCODER_A_LEFT) == LOW)
    {
        if (digitalRead(ENCODER_B_LEFT) == LOW)
        {
            encoderVal_Left--;
        }
        else
        {
            encoderVal_Left++;
        }
    }
    else
    {
        if (digitalRead(ENCODER_B_LEFT) == LOW)
        {
            encoderVal_Left++;
        }
        else
        {
            encoderVal_Left--;
        }
    }
}
void getEncoder_R(void) 
{
    if (digitalRead(ENCODER_A_RIGHT) == LOW)
    {
        if (digitalRead(ENCODER_B_RIGHT) == LOW)
        {
            encoderVal_Right--;
        }
        else
        {
            encoderVal_Right++;
        }
    }
    else
    {
        if (digitalRead(ENCODER_B_RIGHT) == LOW)
        {
            encoderVal_Right++;
        }
        else
        {
            encoderVal_Right--;
        }
    }
}
