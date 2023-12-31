%PLA或聚乳酸材料1.24 g /cm3
% p = 1.24;
%假设摆杆长20cm，截面5*2cm
%假设动量轮直径15cm,厚度2cm
%电机质量20g
%电机直径26.3mm
m = 0.017;%p*20*5*2*0.001;%摆杆质量 kg
M = 0.07;%pi*square(15/2)*2*p*0.001;%动量轮质量 kg
Mm = 20*0.001;%电机质量 kg
L = 0.12;%20*0.01;%摆杆长度 m
R = 0.065;%15/2*0.01;%动量轮半径 m
Rm = 26.3/2*0.01;%电机半径 m
g = 9.8;%重力系数 m/s2
%角度反馈,Kp调到0.12才出现谐振现象，进一步Kp取0.115836时发
%现仿真倒立摆能保持静止,大于之出现振荡，小于之角度不可控

%30deg初始值
%Kp取0.116，Kd取0.005，theta1能得到一个比较好的响应效果，
%能在两秒内收敛到0.0029rad左右（接近0.16deg）
%theta1角速度有一定超调，峰值在1.4rad/s(80deg/s左右)，认
%为可以接受
%20deg初始值，theta1超调在1rad/s以内
%3deg初始值（平衡点附近）一秒左右收敛到可接受的范围
%40deg初始值，theta1超调接近2rad/s，运行较快
%更多的角度测试应等到装置上实际测试
theta1_ini = pi*(20)/180;%theta1初始值

%加入积分量会导致振荡以及不稳定
%dk模型使用的初始值
kp0 = 0.116;%1.8 增大P缩短调节时间，容易振荡，超调值减小
ki0 = 0;%1 增大超调，略微缩短调节时间，易振荡
kd0 = 0.005;%0.8 增大D延长调节时间，减缓振荡
%能接受的“高速”的转速10r/3.63s = 10*2*pi/3.63s = 17.3091rad/s