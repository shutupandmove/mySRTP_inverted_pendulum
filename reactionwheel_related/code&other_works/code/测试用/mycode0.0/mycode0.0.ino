#include <SimpleFOC.h>
#include <math.h>

#define DEF_ANG_PID_RAMP 1000.0f

#define MOTOR_POLARPAIR 7
#define MOTOR_PHASE_R 7.1f

//电机实例初始化
BLDCMotor motor = BLDCMotor(MOTOR_POLARPAIR, MOTOR_PHASE_R);//KV值还未被确定，DC电流力矩模式不常使用
//驱动器实例初始化
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 10, 6, 8);
//编码器实例初始化
MagneticSensorAnalog sensor = MagneticSensorAnalog(A1, 14, 1020);
//电流检测
InlineCurrentSense current_sense = InlineCurrentSense(0.01, 50.0, A0, A2);
//角度PID
PIDController angle_pid(1,2,3,DEF_ANG_PID_RAMP,DEF_POWER_SUPPLY/MOTOR_PHASE_R);//P,I,D,ramp,limit参数都还没定

float angle_tar = 0,dc_current_sp = 0;//目标角度、控制变量初始化

void setup() {
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

  // 电流检测初始化硬件
  current_sense.linkDriver(&driver); 
  current_sense.init();
  // 连接电流传感器和电机
  motor.linkCurrentSense(&current_sense);

  //FOC调制部分根据例程似乎没有必要进行
  
  //传感器与电机校正根据例程似乎没有必要进行

  // 设置力矩模式：
  motor.torque_controller = TorqueControlType::dc_current; 
  
  // 设置运动控制环——这个可能要自己写的！！！
  motor.controller = MotionControlType::torque;

  // 电流控制参数 (Arduino UNO/Mega)——是电机控制环上的PID，似乎与我们要写的控制器独立
  motor.PID_current_q.P = 5;//参数是我已经调过的，能用水平
  motor.PID_current_q.I= 300;
  motor.LPF_current_q.Tf = 0.01; 

  // 初始化电机
  motor.init();
  // 校准编码器，启用FOC
  motor.initFOC();

  delay(1000);
}

void loop() {
  //角度PID控制器处理误差得到控制变量，即设定的直流电流目标值
  dc_current_sp = angle_pid(abs(angle_tar-sensor.getSensorAngle()));//取正好像需要照着模型来
  
  // 运动控制函数
  motor.move(dc_current_sp);//——写入new_target（直流电流力矩下使(q电流)current_sp=target）参量
  
  // FOC算法主函数
  motor.loopFOC();//——对于力矩控制，先通过电磁角度读取DC电流，滤波反馈，和current_sp做差再导入PID控制器输出电压给电机

}
