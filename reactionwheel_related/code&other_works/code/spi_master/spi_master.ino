#include <SPI.h>
#include <SimpleFOC.h>
#include <math.h>

#define DEF_ANG_PID_LIMT 12//待定
#define DEF_ANG_PID_RAMP 1000.0f//待定
#define pi 3.14//圆周率
#define edge pi/3//PID使用区间的半程
#define accr 0.05//控制精度

//驱动器实例初始化
//BLDCDriver3PWM driver = BLDCDriver3PWM(5, 10, 6, 8);
//电机实例初始化
//BLDCMotor motor = BLDCMotor(7);//KV值还未被确定，DC电流力矩模式不常使用
//编码器实例初始化
MagneticSensorI2C pidsensor = MagneticSensorI2C(0x36,12,0X0E,4);
// instantiate the commander simplefoc studio
//Commander command = Commander(Serial);
//电流检测
//InlineCurrentSense current_sense = InlineCurrentSense(0.01, 50.0, A0, A2);

//角度PID
//40_0_6
PIDController angle_pid(5,0,0,DEF_ANG_PID_RAMP,DEF_ANG_PID_LIMT);//P,I,D,ramp,limit参数都还没定
//angle变化范围限定在-3.81---0.67
float angle_tar1 = -2.24,angle_tar2 = -2.24,dc_current_sp = 0;//目标角度(弧度制)、控制变量初始化
float KD = 0;
//simplefoc studio
//void doMotor(char* cmd) { command.motor(&motor, cmd); }

void setup () {

  // 初始化编码传感器硬件
  pidsensor.init();

  Serial.begin(9600); //初始化串口波特率

  //SPI设置
  digitalWrite(SS, HIGH); // 禁用从设备HIGH为禁用
  SPI.begin ();
  SPI.setClockDivider(SPI_CLOCK_DIV8);//设置SPI的时钟为八分之一

  angle_tar1 = pidsensor.getAngle() - pi;
  angle_tar2 = angle_tar1 + 2*pi;
  
  Serial.println("Ready!");
  Serial.print("SENSOR\t");
  Serial.print("T1_U\t");
  Serial.print("TAR1\t");
  Serial.print("T1_D\t");
  Serial.print("T2_U\t");
  Serial.print("TAR2\t");
  Serial.print("T2_D\n");
  delay(1000);  
}
int loop_count = 0;

void loop () {//50
  if (loop_count++ > 25){
   
    pidsensor.update();
    
    Serial.print(pidsensor.getAngle());
    Serial.print("\t");
    Serial.print(angle_tar1+accr);
    Serial.print("\t");
    Serial.print(angle_tar1);
    Serial.print("\t");
    Serial.print(angle_tar1-accr);
    Serial.print("\t");
    Serial.print(angle_tar2+accr);
    Serial.print("\t");
    Serial.print(angle_tar2);
    Serial.print("\t");
    Serial.print(angle_tar2-accr);
    Serial.print("\n");
    
    //角度PID控制器处理误差得到控制变量，即设定的直流电流目标值
//    if((pidsensor.getAngle()>=(angle_tar1-accr))&&(pidsensor.getAngle()<=(angle_tar1+accr))){
//      dc_current_sp = 0;
//    }else 
    if((pidsensor.getAngle()>=(angle_tar1-edge))&&(pidsensor.getAngle()<=(angle_tar1+edge))){
      dc_current_sp = angle_pid(angle_tar1-pidsensor.getAngle())+KD*pidsensor.getVelocity();
    }
    else if((pidsensor.getAngle()>=(angle_tar2-edge))&&(pidsensor.getAngle()<=(angle_tar2+edge))){
      dc_current_sp = angle_pid(angle_tar2-pidsensor.getAngle());
    }else{
//      dc_current_sp = 0;
      dc_current_sp = _sign(pidsensor.getVelocity())*DEF_ANG_PID_LIMT*0.3;
    }
    
    int temp = map(100*dc_current_sp,-DEF_ANG_PID_LIMT*100,DEF_ANG_PID_LIMT*100,0,255);
    
    //SPI传输
    digitalWrite(SS, LOW); //启用从设备
    // 发送测试的字符串  
    SPI.transfer (temp);
//    Serial.print(dc_current_sp);
//    Serial.print("\t");
//    Serial.println(temp);
    digitalWrite(SS, HIGH); // 发送完毕后再次禁用从设备
    loop_count = 0;
  }
}
