#include "Arduino.h"
#include "QFOC.h"
#include "WebTuning.h"
#include "BleKeyboard.h"
#include "OneButton.h"

Motor BLDC;
BleKeyboard bleKeyboard;
OneButton button(4,true,true);
const char* ssid = "屈圣桥的iPhone";
const char* pwd = "qsq060823";
WebServer server(PORT);

float KKP = 0.02;
void handleSetPID()
{
    if (server.hasArg("channel")&&server.hasArg("p")&&server.hasArg("i")&&server.hasArg("d")&&server.hasArg("limit")) 
    {
        float target = server.arg("channel").toInt();
        float kp = server.arg("p").toFloat();
        float ki = server.arg("i").toFloat();
        float kd = server.arg("d").toFloat();
        if(target==1)  BLDC.SetPID_Angel(kp,ki,kd);
        if(target==2)  BLDC.SetPID_Velocity(kp,ki,kd);
        if(target==3)  {KKP=kp;Serial.print(KKP);}
        server.send(200, "text/plain", "Target set: " + String(target)+"  Kp="+String(kp)+" Ki="+String(ki)+" Kd="+String(kd));
    } 
    else 
    {
        server.send(400, "text/plain", "Missing target parameter");
    }
}
void handleRoot() 
{
    server.send(200, "text/plain", "QSQ's WebPIDTuner Ready");
}


float DEG2RAD(float DEG)
{
    return DEG/180.0*PI;
}

float RAD2DEG(float RAD)
{
    return RAD*180.0/PI;
}

float rd;
float attractor = 10*PI/180.0;


void BTN_Click()
{
    attractor = 1*PI/180.0;
    Serial.println("click\n");
    bleKeyboard.pressDial();
    bleKeyboard.releaseDial();
}

void BTN_LongPressBegin()
{
    attractor = 10*PI/180.0;
    Serial.println("LPB\n");
    bleKeyboard.pressDial();
}

void BTN_LongPressEnd()
{
    Serial.println("LPE");
    bleKeyboard.releaseDial();
}

void button_event_init() {
  button.reset();               // 清除一下按钮状态机的状态
  button.setDebounceTicks(80);  // 设置消抖时长为80毫秒,默认值为：50毫秒
  button.setClickTicks(400);    // 设置单击时长为500毫秒,默认值为：400毫秒
  button.setPressTicks(1000);   // 设置长按时长为1000毫秒,默认值为：800毫秒

  button.attachClick(BTN_Click);                    // 初始化单击回调函数
  button.attachLongPressStart(BTN_LongPressBegin);  // 初始化长按开始回调函数
  button.attachLongPressStop(BTN_LongPressEnd);    // 初始化长按结束回调函数
}

float preAngle=0;
void Knob()
{
    //BLDC.SetVelocityAngel(serial_motor_target());
    rd = round(BLDC.GetAngel()/attractor)*attractor;
    BLDC.SetTorque(KKP*(rd - BLDC.GetAngel()));

    if(BLDC.GetAngel()-preAngle>DEG2RAD(30)) 
    {
        Serial.print("rotate");
        bleKeyboard.rotate(1);
        preAngle = BLDC.GetAngel();
    }
    else if (BLDC.GetAngel()-preAngle<-DEG2RAD(30))
    {
        Serial.print("rotate");
        bleKeyboard.rotate(-1);
        preAngle = BLDC.GetAngel();
    }
    
}

void loop() 
{     
    button.tick(); // 让OneButton库处理按钮状态
    Knob();
    //设置PID
//   DFOC_M0_SET_ANGLE_PID(1.8,0,0,0);
//   DFOC_M0_SET_VEL_PID(0.01,0.00,0,0);
//   //给库设定速度
//   DFOC_M0_set_Velocity_Angle(serial_motor_target()); 
//   //接收串口
    //serialReceiveUserCommand();
    //server.handleClient();
    // 
    
    // bleKeyboard.pressDial();
    // delay(1);
    // bleKeyboard.rotate(1);
    // delay(1);
    // bleKeyboard.releaseDial();
    // delay(1);
    // bleKeyboard.pressDial();
    // Serial.println("press");
    // delay(1000);
    // bleKeyboard.releaseDial();
    // bleKeyboard.rotate(1);
    delay(100);


    //BLDC.SetVelocityAngel(BLDC.GetAngel());
    //BLDC.SetForceAngel(serial_motor_target());
}

void setup() {
  Serial.begin(115200);
  Serial.print("ws");
//   WIFI_Init( ssid,pwd);
  BLDC.MotorInit(1,32,33,25,0,1,2,12.6,7,1,22,21);
  BLDC.SetLowPassFliter_Tf(0.6);
//  BLDC1.MotorInit(2,13,12,13,3,4,5,12.6,7,1,4,2);
  Serial.print("Ssssssss");
  BLDC.SetPID_Angel(0.06,0.0004,0.0003,100,0);
  BLDC.SetPID_Velocity(1.2,0.00,0,6.3,0);
//   server.on("/",handleRoot);
//   server.on("/setpid",handleSetPID);
//   server.begin();
//   Serial.println("HTTP server started");
  button_event_init();
  bleKeyboard.begin();
}