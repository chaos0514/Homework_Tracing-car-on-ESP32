#include <Arduino.h>

hw_timer_t *Pid_tim = NULL;
hw_timer_t *Motor_tim = NULL;
hw_timer_t *Trace_tim = NULL;

const int Motor_channel_Left = 14;
const int Motor_channel_Right = 15;
int Motor_freq_PWM = 500;
int Motor_resolution_PWM = 10;
/*******************/
#define TRACE_MID 21
#define TRACE_LEFT_1 25
#define TRACE_LEFT_2 26
#define TRACE_LEFT_3 27
#define TRACE_RIGHT_1 13
#define TRACE_RIGHT_2 16
#define TRACE_RIGHT_3 17
#define velocity_max 20
/*******************/
#define ENCODER_A_LEFT 32
#define ENCODER_B_LEFT 33
#define ENCODER_A_RIGHT 34
#define ENCODER_B_RIGHT 35
#define PWML 22
#define PWMR 23
#define PERIOD 20
/******************/
int Target_Left = 5;
int encoderVal_Left;
int velocity_Left;
int Left_Error;
int Left_Error_I;
int Left_Error_Sum;
int Output_Left;
/*******************/
int Target_Right = 5;
int encoderVal_Right;
int velocity_Right;
int Right_Error;
int Right_Error_I;
int Right_Error_Sum;
int Output_Right;

int kp = 10, ti = 0, td = 0, error_max = 0, t = 20;

void Motor_Init();
void IRAM_ATTR getEncoder_L();
void IRAM_ATTR getEncoder_R();
void IRAM_ATTR Pid_TimeEvent();
void Trace_Init();
void IRAM_ATTR Trace_TimeEvent();
void Target_Left_Cal();
void Target_Right_Cal();

void setup()
{
  Serial.begin(9600);
  Motor_Init(); // 电机初始化
  //   ServoInit(); //机械臂初始化
  //   Trace_Init(); // 红外初始化
  // /********位置环**********/
  //   Trace_tim = timerBegin(0, 80, true); // 0号时钟每20ms读取循迹路线
  //   timerAttachInterrupt(Trace_tim, &Trace_TimeEvent, true);
  //   timerAlarmWrite(Trace_tim, 20000, true);
  //   timerAlarmEnable(Trace_tim);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_LEFT), getEncoder_L, CHANGE);  // 通过左电机A相的电平变化触发外部中断，得到脉冲
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_RIGHT), getEncoder_R, CHANGE); // 通过右电机A相的电平变化触发外部中断，得到脉冲
}

void loop()
{
  // Serial.println("velocity_Left:");
  //  Serial.println(velocity_Left);
  // Serial.println("\t");
  Serial.print("velocity_Right:");
  Serial.println(velocity_Right);
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
  delay(100);
}

void Motor_Init(void)
{
  pinMode(ENCODER_A_RIGHT, INPUT);
  pinMode(ENCODER_B_RIGHT, INPUT);
  pinMode(PWMR, OUTPUT);
  pinMode(PWML, OUTPUT);
  Pid_tim = timerBegin(1, 80, true); // 1号时钟每20ms执行一次pid算法并且驱动电机
  timerAttachInterrupt(Pid_tim, &Pid_TimeEvent, true);
  timerAlarmWrite(Pid_tim, 20000, true);
  timerAlarmEnable(Pid_tim);
  ledcSetup(Motor_channel_Left, Motor_freq_PWM, Motor_resolution_PWM);
  ledcSetup(Motor_channel_Right, Motor_freq_PWM, Motor_resolution_PWM);
  ledcAttachPin(PWML, Motor_channel_Left);
  ledcAttachPin(PWMR, Motor_channel_Right);
}
void IRAM_ATTR Pid_TimeEvent()
{
  velocity_Right = encoderVal_Right / 20;
  Right_Error_I = Right_Error;
  Right_Error = Target_Right - velocity_Right;
  Right_Error_Sum += Right_Error;
  Right_Error_Sum = Right_Error_Sum > error_max ? error_max : Right_Error_Sum;
  Right_Error_Sum = Right_Error_Sum < -error_max ? -error_max : Right_Error_Sum;
  Output_Right = kp * Right_Error + ti * Right_Error_Sum + td * (Right_Error - Right_Error_I);
  Output_Right = Output_Right > 1023 ? 1023 : Output_Right;
  Output_Right = Output_Right < -1023 ? -1023 : Output_Right;
  ledcWrite(Motor_channel_Right, Output_Right);
  encoderVal_Right = 0;

  velocity_Left = encoderVal_Left / 20;
  Left_Error_I = Left_Error;
  Left_Error = Target_Left - velocity_Left;
  Left_Error_Sum += Left_Error;
  Left_Error_Sum = Left_Error_Sum > error_max ? error_max : Left_Error_Sum;
  Left_Error_Sum = Left_Error_Sum < -error_max ? -error_max : Left_Error_Sum;
  Output_Left = kp * Left_Error + ti * Left_Error_Sum + td * (Left_Error - Left_Error_I);
  Output_Left = Output_Left > 1023 ? 1023 : Output_Left;
  Output_Left = Output_Left < -1023 ? -1023 : Output_Left;
  ledcWrite(Motor_channel_Left, Output_Left);
  encoderVal_Left = 0;
}

// void Trace_Init()
// {
//   pinMode(TRACE_MID, INPUT);
//   pinMode(TRACE_LEFT_1, INPUT);
//   pinMode(TRACE_LEFT_2, INPUT);
//   pinMode(TRACE_LEFT_3, INPUT);
//   pinMode(TRACE_RIGHT_1, INPUT);
//   pinMode(TRACE_RIGHT_2, INPUT);
//   pinMode(TRACE_RIGHT_3, INPUT);
// }
// void IRAM_ATTR Trace_TimeEvent()
// {
//   Target_Left_Cal();
//   Target_Right_Cal();
// }
// void Target_Left_Cal()
// {
//   Target_Left_Temp = Target_Left;
//   Target_Right_Temp = Target_Right;
//   if (digitalRead(TRACE_MID) == LOW)
//   {
//     Target_Left = velocity_max;
//     Target_Right = velocity_max;
//   }
//   if (digitalRead(TRACE_LEFT_1) == LOW) // 轻微轻微右转
//   {
//     Target_Left = 0.9 * velocity_max;
//     Target_Right = 0.8 * velocity_max;
//   }
//   if (digitalRead(TRACE_LEFT_2) == LOW) // 右转
//   {
//     Target_Left = 0.9 * velocity_max;
//     Target_Right = 0.6 * velocity_max;
//   }
//   if (digitalRead(TRACE_LEFT_3) == LOW) // 90°右转
//   {
//     Target_Left = 1 * velocity_max;
//     Target_Right = -1 * velocity_max;
//   }
//   Target_Left = Target_Left * 0.6 + Target_Left_Temp * 0.8;
//   Target_Right = Target_Right * 0.6 + Target_Left_Temp * 0.8;
// }
// void Target_Right_Cal()
// {
//   Target_Left_Temp = Target_Left;
//   Target_Right_Temp = Target_Right;
//   if (digitalRead(TRACE_MID) == LOW)
//   {
//     Target_Left = velocity_max;
//     Target_Right = velocity_max;
//   }
//   if (digitalRead(TRACE_RIGHT_1) == LOW) // 轻微轻微左转
//   {
//     Target_Left = 0.8 * velocity_max;
//     Target_Right = 0.9 * velocity_max;
//   }
//   if (digitalRead(TRACE_RIGHT_2) == LOW) // 左转
//   {
//     Target_Left = 0.6 * velocity_max;
//     Target_Right = 0.9 * velocity_max;
//   }
//   if (digitalRead(TRACE_RIGHT_3) == LOW) // 90°左转
//   {
//     Target_Left = -1 * velocity_max;
//     Target_Right = 1 * velocity_max;
//   }
//   Target_Left = Target_Left * 0.6 + Target_Left_Temp * 0.8;
//   Target_Right = Target_Right * 0.6 + Target_Left_Temp * 0.8;
// }

/*
请注意！！！
读取编码器的脉冲一定要考虑电机的转向
转向不同时，编码器digitalRead()所判断的条件相反
否则电机会失控
*/
void IRAM_ATTR getEncoder_L(void)
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
void IRAM_ATTR getEncoder_R(void)
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
