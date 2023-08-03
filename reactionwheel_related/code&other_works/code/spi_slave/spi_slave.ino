#include <SPI.h>
#include <SimpleFOC.h>
#include <math.h>

#define DEF_ANG_PID_LIMT 12//待定

//SPI初始量
int buff;
volatile boolean process;
float target_voltage = 0;

//电机实例初始化
BLDCMotor motor = BLDCMotor(7);//KV值还未被确定，DC电流力矩模式不常使用
//驱动器实例初始化
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 10, 6, 8);
//编码器实例初始化
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

float dc_current_sp = 0;//目标角度、控制变量初始化
//int tick = 0;
// instantiate the commander
Commander command = Commander(Serial);

//simplefoc studio
void doMotor(char* cmd) { command.motor(&motor, cmd); }

void setup () {

  // 初始化编码传感器硬件
  sensor.init();
  // 连接电机和传感器
  motor.linkSensor(&sensor);

  // 配置驱动器
  // 电源电压 [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // 连接驱动器和电机
  motor.linkDriver(&driver);

  // aligning voltage 
  motor.voltage_sensor_align = 5;
  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // set motion control loop to be used
  motor.controller = MotionControlType::torque;

  Serial.begin (9600);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // 初始化电机
  motor.init();
  // 校准编码器，启用FOC
  motor.initFOC();

  //simplefoc studio
  command.add('M', doMotor, "motor");
  Serial.println(F("Motor ready."));

  //SPI初始化
  pinMode(MISO, OUTPUT); //将MISO设置为输出以便数据发送主机
  SPCR |= _BV(SPE); //在从机模式下打开SPI通讯
  process = false;
  SPI.attachInterrupt(); //打开中断
  
  _delay(1000);
}
 
 
ISR (SPI_STC_vect) // SPI中断程序
{ 
  buff = SPDR; // 从SPI数据寄存器读取字节
  process = true;
}
 
void loop () {
  // FOC算法主函数
  motor.loopFOC();//——对于力矩控制，先通过电磁角度读取DC电流，滤波反馈，和current_sp做差再导入PID控制器输出电压给电机


  if (process) {
    process = false; //重置通讯过程
    buff = SPDR;
    target_voltage = map(buff,0,255,-DEF_ANG_PID_LIMT*100,DEF_ANG_PID_LIMT*100)*0.01;
    // 运动控制函数
    motor.move(target_voltage);//——写入new_target（直流电流力矩下使(q电流)current_sp=target）参量

    Serial.print(buff);
    Serial.print('\t');
    Serial.println(target_voltage); //在串口监视器上打印接收到的buff数据
  }
  command.run();

}
