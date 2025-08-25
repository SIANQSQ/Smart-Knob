#include <Arduino.h> 
#include "QFOC.h"

#define pwmFreq 30000  //30000HZ pwm频率
#define pwmReso 8 //8bit pwm精度
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))


#define _3PI_2 4.71238898038f
//float zero_electric_angle=0;
//int PP=1,DIR=1;


// //低通滤波初始化
// LowPassFilter M0_Vel_Flt = LowPassFilter(0.01); // Tf = 10ms   //M0速度环
// //PID
// PIDController vel_loop_M0 = PIDController{.P = 2, .I = 0, .D = 0, .ramp = 100000, .limit = voltage_power_supply/2};
// PIDController angle_loop_M0 = PIDController{.P = 2, .I = 0, .D = 0, .ramp = 100000, .limit = 100};

// //AS5600
// Sensor_AS5600 S0=Sensor_AS5600(0);
// TwoWire S0_I2C = TwoWire(0);

//=================PID 设置函数=================
//速度PID


// //M0速度PID接口
// float DFOC_M0_VEL_PID(float error)   //M0速度环
// {
//    return vel_loop_M0(error);
   
// }
// //M0角度PID接口
// float DFOC_M0_ANGLE_PID(float error)
// {
//   return angle_loop_M0(error);
// }


//初始变量及函数定义
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
//宏定义实现的一个约束函数,用于限制一个值的范围。
//具体来说，该宏定义的名称为 _constrain，接受三个参数 amt、low 和 high，分别表示要限制的值、最小值和最大值。该宏定义的实现使用了三元运算符，根据 amt 是否小于 low 或大于 high，返回其中的最大或最小值，或者返回原值。
//换句话说，如果 amt 小于 low，则返回 low；如果 amt 大于 high，则返回 high；否则返回 amt。这样，_constrain(amt, low, high) 就会将 amt 约束在 [low, high] 的范围内。1


// 归一化角度到 [0,2PI]
float _normalizeAngle(float angle){
  float a = fmod(angle, 2*PI);   //取余运算可以用于归一化，列出特殊值例子算便知
  return a >= 0 ? a : (a + 2*PI);  
  //三目运算符。格式：condition ? expr1 : expr2 
  //其中，condition 是要求值的条件表达式，如果条件成立，则返回 expr1 的值，否则返回 expr2 的值。可以将三目运算符视为 if-else 语句的简化形式。
  //fmod 函数的余数的符号与除数相同。因此，当 angle 的值为负数时，余数的符号将与 _2PI 的符号相反。也就是说，如果 angle 的值小于 0 且 _2PI 的值为正数，则 fmod(angle, _2PI) 的余数将为负数。
  //例如，当 angle 的值为 -PI/2，_2PI 的值为 2PI 时，fmod(angle, _2PI) 将返回一个负数。在这种情况下，可以通过将负数的余数加上 _2PI 来将角度归一化到 [0, 2PI] 的范围内，以确保角度的值始终为正数。
}





//==============串口接收==============
float motor_target;
int commaPosition;
String serialReceiveUserCommand() {
  
  // a string to hold incoming data
  static String received_chars;
  
  String command = "";

  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the string buffer:
    received_chars += inChar;

    // end of user input
    if (inChar == '\n') {
      
      // execute the user command
      command = received_chars;

      commaPosition = command.indexOf('\n');//检测字符串中的逗号
      if(commaPosition != -1)//如果有逗号存在就向下执行
      {
          motor_target = command.substring(0,commaPosition).toDouble();            //电机角度
          Serial.println(motor_target);
      }
      // reset the command buffer 
      received_chars = "";
    }
  }
  return command;
}


float serial_motor_target()
{
  return motor_target;
}


/*
//================简易接口函数================
void DFOC_M0_set_Velocity_Angle(float Target)
{
 setTorque(DFOC_M0_VEL_PID(DFOC_M0_ANGLE_PID((Target-DFOC_M0_Angle())*180/PI)),_electricalAngle());   //角度闭环
}

void DFOC_M0_setVelocity(float Target)
{
  setTorque(DFOC_M0_VEL_PID((serial_motor_target()-DFOC_M0_Velocity())*180/PI),_electricalAngle());   //速度闭环
}

void DFOC_M0_set_Force_Angle(float Target)   //力位
{
  setTorque(DFOC_M0_ANGLE_PID((Target-DFOC_M0_Angle())*180/PI),_electricalAngle());
}

void DFOC_M0_setTorque(float Target)
{
  setTorque(Target,_electricalAngle());
}

*/



void Motor::SetPID_Angel(float Kp,float Ki,float Kd)
{
    PID_Angel.P=Kp;
    PID_Angel.I=Ki;
    PID_Angel.D=Kd;
}
void Motor::SetPID_Angel(float Kp,float Ki,float Kd,float _limit)
{
    PID_Angel.P=Kp;
    PID_Angel.I=Ki;
    PID_Angel.D=Kd;
    PID_Angel.limit=_limit;
}
void Motor::SetPID_Angel(float Kp,float Ki,float Kd,float _limit,float _ramp)
{
    PID_Angel.P=Kp;
    PID_Angel.I=Ki;
    PID_Angel.D=Kd;
    PID_Angel.limit=_limit;
    PID_Angel.output_ramp = _ramp;
}

void Motor::SetPID_Velocity(float Kp,float Ki,float Kd)
{
    PID_Velocity.P=Kp;
    PID_Velocity.I=Ki;
    PID_Velocity.D=Kd;
}

void Motor::SetPID_Velocity(float Kp,float Ki,float Kd,float _limit,float _ramp)
{
    PID_Velocity.P=Kp;
    PID_Velocity.I=Ki;
    PID_Velocity.D=Kd;
    PID_Velocity.limit=_limit;
    PID_Velocity.output_ramp = _ramp;
}

void Motor::SetMotorTorque(float Uq,float angle_el) {
  AS5600.Sensor_update(); //更新传感器数值
  Uq=_constrain(Uq,-(PowerSupply)/2,(PowerSupply)/2);
  float Ud=0;
  angle_el = _normalizeAngle(angle_el);
  // 帕克逆变换
  Ualpha =  -Uq*sin(angle_el); 
  Ubeta =   Uq*cos(angle_el); 

  // 克拉克逆变换
  Ua = Ualpha + PowerSupply/2;
  Ub = (sqrt(3)*Ubeta-Ualpha)/2 + PowerSupply/2;
  Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + PowerSupply/2;
  SetPwm(Ua,Ub,Uc);
  //Serial.Print("InTorq");
}

void Motor::MotorInit(int _ID,int _pwmA,int _pwmB,int _pwmC,int _pwmChannelA,int _pwmChannelB,int _pwmChannelC,float _PowerSupply,int _PolePairs,int _SensorDIR,int _I2C_SDA,int _I2C_SCL)
{
    ID=_ID;
    pwmA = _pwmA;
    pwmB = _pwmB;
    pwmC = _pwmC;
    pwmChannelA=_pwmChannelA;
    pwmChannelB=_pwmChannelB;
    pwmChannelB=_pwmChannelC;
    PolePairs = _PolePairs;
    SensorDIR = _SensorDIR;
    PowerSupply = _PowerSupply;
    I2C_SDA = _I2C_SDA;
    I2C_SCL = _I2C_SCL;

    pinMode(pwmA, OUTPUT);
    pinMode(pwmB, OUTPUT);
    pinMode(pwmC, OUTPUT);
    ledcSetup(_pwmChannelA, pwmFreq, pwmReso);  //pwm频道, 频率, 精度
    ledcSetup(_pwmChannelB, pwmFreq, pwmReso);  
    ledcSetup(_pwmChannelC, pwmFreq, pwmReso);  
    ledcAttachPin(pwmA, _pwmChannelA);
    ledcAttachPin(pwmB, _pwmChannelB);
    ledcAttachPin(pwmC, _pwmChannelC);
    Serial.println("完成PWM初始化设置");

    //AS5600
    IIC.begin(I2C_SDA,I2C_SCL, 400000UL);
    AS5600.Sensor_init(&IIC);   //初始化编码器0
    Serial.println("编码器加载完毕");
    
    SetMotorTorque(3, _3PI_2);  //起劲
    delay(1000);
    AS5600.Sensor_update();  //更新角度，方便下面电角度读取
    Zero_ElectricalAngel=GetElectricalAngel();
    SetMotorTorque(0, _3PI_2);  //松劲（解除校准）
    Serial.print("0电角度：");Serial.println(Zero_ElectricalAngel);
  
}

void Motor::SetLowPassFliter_Tf(float _TF)
{
    Fliter.Tf=_TF;
}

// 设置PWM到控制器输出
void Motor::SetPwm(float Ua, float Ub, float Uc) {
  // 限制上限
  Ua = _constrain(Ua, 0.0f, PowerSupply);
  Ub = _constrain(Ub, 0.0f, PowerSupply);
  Uc = _constrain(Uc, 0.0f, PowerSupply);
  // 计算占空比
  // 限制占空比从0到1
  float dc_a = _constrain(Ua / PowerSupply, 0.0f , 1.0f );
  float dc_b = _constrain(Ub / PowerSupply, 0.0f , 1.0f );
  float dc_c = _constrain(Uc / PowerSupply, 0.0f , 1.0f );

//   dc_a = 0.5;
//   dc_b = 0.5;
//   dc_c = 0.5;
  //写入PWM到PWM A B C 通道
  ledcWrite(0, dc_a*255);
  ledcWrite(1, dc_b*255);
  ledcWrite(2, dc_c*255);
//   Serial.printf("%.2f  %.2f  %.2f",dc_a,dc_b,dc_c);
//   Serial.print("\n");
}

float Motor::GetElectricalAngel()
{
    return  _normalizeAngle((float)(SensorDIR *  PolePairs) * AS5600.getMechanicalAngle()-Zero_ElectricalAngel);
}

float Motor::GetAngel()
{
    return SensorDIR*AS5600.getAngle();
}

float Motor::GetMechanicalAngel()
{
    return SensorDIR*AS5600.getMechanicalAngle();
}

float Motor::GetVelocity()
{
    float vel=AS5600.getVelocity();
    float vel_flit=Fliter(SensorDIR*vel);
    return vel_flit;  
}

void Motor::SetForceAngel(float Target)
{
    SetMotorTorque(PID_Angel.Cal((Target-GetAngel())*180/PI),GetElectricalAngel());
}

void Motor::SetVelocityAngel(float Target)
{
    SetMotorTorque(PID_Velocity.Cal(PID_Angel.Cal((Target-GetAngel())*180/PI)),GetElectricalAngel());   //角度闭环
    // Serial.print(GetElectricalAngel());
    // Serial.print("\n");
    // Serial.print(PID_Velocity.Cal(PID_Angel.Cal((Target-GetAngel())*180/PI)));
    
    // // Serial.print(Target-GetAngel());
    //  Serial.print("\n");
}

void Motor::SetTorque(float Uq)
{
    SetMotorTorque(Uq,GetElectricalAngel());
}