#include <Arduino.h>

hw_timer_t *Pid_tim = NULL;
hw_timer_t *Motor_tim = NULL;
hw_timer_t *Trace_tim = NULL;

const int Motor_channel_PWMA = 14;
const int Motor_channel_PWMB = 15;
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

volatile float Target_Left;
volatile float Target_Right;
volatile float Target_Left_Temp;
volatile float Target_Right_Temp;
int32_t encoderVal_Left;
int32_t encoderVal_Right;
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

void Motor_Init(void);
void IRAM_ATTR Pid_TimeEvent();
int pidcontrol_L(float target, float cTemp_Rightrent); // 1的PID算法
int pidcontrol_R(float target, float cTemp_Rightrent); // 2的PID算法
void Trace_Init();
void IRAM_ATTR Trace_TimeEvent();
void Target_Left_Cal();
void Target_Right_Cal();
void IRAM_ATTR getEncoder_L(void);
void IRAM_ATTR getEncoder_R(void);

void setup()
{
  Serial.begin(9600);
  pinMode(ENCODER_A_RIGHT, INPUT);
  pinMode(ENCODER_B_RIGHT, INPUT);
  // Motor_Init(); // 电机初始化
  //   ServoInit(); //机械臂初始化
  //   Trace_Init(); // 红外初始化
  // /********速度环**********/
  Pid_tim = timerBegin(1, 80, true); // 1号时钟每20ms执行一次pid算法并且驱动电机
  timerAttachInterrupt(Pid_tim, &Pid_TimeEvent, true);
  timerAlarmWrite(Pid_tim, 20000, true);
  timerAlarmEnable(Pid_tim);
  // /********位置环**********/
  //   Trace_tim = timerBegin(0, 80, true); // 0号时钟每20ms读取循迹路线
  //   timerAttachInterrupt(Trace_tim, &Trace_TimeEvent, true);
  //   timerAlarmWrite(Trace_tim, 20000, true);
  //   timerAlarmEnable(Trace_tim);

  // attachInterrupt(digitalPinToInterrupt(ENCODER_A_LEFT), getEncoder_L, CHANGE);  // 通过左电机A相的电平变化触发外部中断，得到脉冲
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_RIGHT), getEncoder_R, CHANGE); // 通过右电机A相的电平变化触发外部中断，得到脉冲
}

void loop()
{
  // Serial.println("velocity_Left:");
  //  Serial.println(velocity_Left);
  // Serial.println("\t");
  // Serial.println("velocity_Right:");
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
  ledcSetup(Motor_channel_PWMA, Motor_freq_PWM, Motor_resolution_PWM);
  ledcSetup(Motor_channel_PWMB, Motor_freq_PWM, Motor_resolution_PWM);
  ledcAttachPin(PWML, Motor_channel_PWMA);
  ledcAttachPin(PWMR, Motor_channel_PWMB);
}
void IRAM_ATTR Pid_TimeEvent()
{
  // velocity_Left = (encoderVal_Left / 780) * 3.1415 * 2.0 * (1000 / PERIOD);
  // Output_Left = pidcontrol_L(Target_Left, velocity_Left);
  velocity_Right = (1.0 * encoderVal_Right / 780) * 3.1415 * 2.0 * (1000 / PERIOD);
  // Output_Right = pidcontrol_R(Target_Right, velocity_Right);
  // ledcWrite(PWML, 1.0 / 2048.0 * Output_Left + 0.5);
  // encoderVal_Left = 0;
  // ledcWrite(PWMR, 1.0 / 2048.0 * Output_Right + 0.5);
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
void Trace_Init()
{
  pinMode(TRACE_MID, INPUT);
  pinMode(TRACE_LEFT_1, INPUT);
  pinMode(TRACE_LEFT_2, INPUT);
  pinMode(TRACE_LEFT_3, INPUT);
  pinMode(TRACE_RIGHT_1, INPUT);
  pinMode(TRACE_RIGHT_2, INPUT);
  pinMode(TRACE_RIGHT_3, INPUT);
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
