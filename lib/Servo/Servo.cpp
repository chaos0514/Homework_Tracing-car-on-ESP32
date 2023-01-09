// #include "myServo.h"

// #define CLAW_PWM 18
// #define PITCH_PWM 19

// Servo claw, pitch;

// const int CLAW_CHANNEL = 6;
// const int PITCH_CHANNEL = 7;

// int freq_PWM = 500;
// int resolution_PWM = 10;
// void ServoInit()
// {
//     ledcSetup(CLAW_CHANNEL, freq_PWM, resolution_PWM);   // 设置通道
//     ledcSetup(PITCH_CHANNEL, freq_PWM, resolution_PWM); // 设置通道
//     ledcAttachPin(CLAW_PWM, CLAW_CHANNEL);                // 将 LEDC 通道绑定到指定 IO 口上以实现输出
//     ledcAttachPin(PITCH_PWM, PITCH_CHANNEL);
// }
// void ServoControl()
// {
//     ledcWrite(CLAW_PWM, 1023);
//     ledcWrite(PITCH_PWM, 1023);
// }