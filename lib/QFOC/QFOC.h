#include "AS5600.h"
#include "pid.h"
#include "Wire.h"
#include "lowpass_filter.h"
#ifndef __QFOC_H
#define __QFOC_H

float serial_motor_target();
String serialReceiveUserCommand();
class Motor
{
    public:
        void MotorInit(int _ID,int _pwmA,int _pwmB,int _pwmC,int _pwmChannelA,int _pwmChannelB,int _pwmChannelC,float _PowerSupply,int _PloarPairs,int _SensorDIR,int _I2C_SDA,int _I2C_SCL);
        void SetPID_Angel(float Kp,float Ki,float Kd);
        void SetPID_Angel(float Kp,float Ki,float Kd,float _limit);
        void SetPID_Angel(float Kp,float Ki,float Kd,float limit,float _ramp);
        void SetPID_Velocity(float Kp,float Ki,float Kd);
        void SetPID_Velocity(float Kp,float Ki,float Kd,float _limit);
        void SetPID_Velocity(float Kp,float Ki,float Kd,float limit,float _ramp);
        void SetPID_Current(float Kp,float Ki,float Kd);
        void SetMotorTorque(float Uq,float angle_el);
        void SetLowPassFliter_Tf(float _TF);
        void SetPwm(float Ua, float Ub, float Uc);
        float GetElectricalAngel();
        float GetMechanicalAngel();
        float GetVelocity();
        float GetAngel();

        void SetForceAngel(float target);
        void SetVelocityAngel(float target);
        void SetTorque(float Uq);
    
    private:
        int ID;  //电机ID
        int pwmA,pwmB,pwmC;  //pwm信号引脚
        int pwmChannelA,pwmChannelB,pwmChannelC;  //pwm通道
        int I2C_SDA,I2C_SCL;  //AS5600 I2C引脚
        int PolePairs;  //电机极对数
        int SensorDIR;    //编码器方向
        float AngelKp,AngelKi,AngelKd;  //角度环PID参数
        float VelocityKp,VelocityKi,VelocityKd;  //速度环PID参数
        float Vel;  //速度
        float MechanicalAngel;  //机械角度
        float ElectricalAngel;  //电角度
        float Zero_ElectricalAngel = 0;  //零电角度
        PIDController PID_Angel;  //角度环PID控制器
        PIDController PID_Velocity;  //速度环PID控制器
        PIDController PID_Current;  //电流环PID控制器
        Sensor_AS5600 AS5600;  //编码器对象
        TwoWire IIC = TwoWire(0);
        LowPassFilter Fliter = LowPassFilter(0.01);
        float PowerSupply;

        float Ualpha,Ubeta=0,Ua=0,Ub=0,Uc=0;
        
};


#endif