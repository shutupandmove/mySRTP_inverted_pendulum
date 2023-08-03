 //BLDCMotor.cpp文件

// Iterative function looping FOC algorithm, setting Uq on the Motor
 // The faster it can be run the better
 void BLDCMotor::loopFOC() {
   // if disabled do nothing
   if(!enabled) return; 
   // if open-loop do nothing
   if( controller==MotionControlType::angle_openloop || controller==MotionControlType::velocity_openloop ) return;
  
   // shaft angle
   shaft_angle = shaftAngle();
   // electrical angle - need shaftAngle to be called first
   electrical_angle = electricalAngle();
  
   switch (torque_controller) {
     case TorqueControlType::voltage:
       // no need to do anything really
       break;
     case TorqueControlType::dc_current:
       if(!current_sense) return;
       // read overall current magnitude
       current.q = current_sense->getDCCurrent(electrical_angle);
       // filter the value values
       current.q = LPF_current_q(current.q);
       // calculate the phase voltage
       voltage.q = PID_current_q(current_sp - current.q); 
       voltage.d = 0;
       break;
     case TorqueControlType::foc_current:
       if(!current_sense) return;
       // read dq currents
       current = current_sense->getFOCCurrents(electrical_angle);
       // filter values
       current.q = LPF_current_q(current.q);
       current.d = LPF_current_d(current.d);
       // calculate the phase voltages
       voltage.q = PID_current_q(current_sp - current.q); 
       voltage.d = PID_current_d(-current.d);
       break;
     default:
       // no torque control selected
       if(monitor_port) monitor_port->println(F("MOT: no torque control selected!"));
       break;
   }
   
   // set the phase voltage - FOC heart function :)
   setPhaseVoltage(voltage.q, voltage.d, electrical_angle);
 }

// get current magnitude 
 //   - absolute  - if no electrical_angle provided 
 //   - signed    - if angle provided
 float CurrentSense::getDCCurrent(float motor_electrical_angle){
     // read current phase currents
     PhaseCurrent_s current = getPhaseCurrents();
     // currnet sign - if motor angle not provided the magnitude is always positive
     float sign = 1;
  
     // calculate clarke transform
     float i_alpha, i_beta;
     if(!current.c){
         // if only two measured currents
         i_alpha = current.a;  
         i_beta = _1_SQRT3 * current.a + _2_SQRT3 * current.b;
     }else{
         i_alpha = 2*(current.a - (current.b - current.c))/3.0;    
         i_beta = _2_SQRT3 *( current.b  - current.c );
     }
  
     // if motor angle provided function returns signed value of the current
     // determine the sign of the current
     // sign(atan2(current.q, current.d)) is the same as c.q > 0 ? 1 : -1  
     if(motor_electrical_angle) 
         sign = (i_beta * _cos(motor_electrical_angle) - i_alpha*_sin(motor_electrical_angle)) > 0 ? 1 : -1;  
     // return current magnitude
     return sign*_sqrt(i_alpha*i_alpha + i_beta*i_beta);
 }
 
 // Iterative function running outer loop of the FOC algorithm
 // Behavior of this function is determined by the motor.controller variable
 // It runs either angle, velocity or torque loop
 // - needs to be called iteratively it is asynchronous function
 // - if target is not set it uses motor.target value
 void BLDCMotor::move(float new_target) {
   // if disabled do nothing
   if(!enabled) return; 
   // downsampling (optional)
   if(motion_cnt++ < motion_downsample) return;
   motion_cnt = 0;
   // set internal target variable
   if(_isset(new_target)) target = new_target;
   // get angular velocity
   shaft_velocity = shaftVelocity();
  
   switch (controller) {
     case MotionControlType::torque:
       if(torque_controller == TorqueControlType::voltage) // if voltage torque control
         if(!_isset(phase_resistance))  voltage.q = target;
         else voltage.q =  target*phase_resistance; 
       else 
         current_sp = target; // if current/foc_current torque control
       break;
     case MotionControlType::angle:
       // angle set point
       shaft_angle_sp = target;
       // calculate velocity set point
       shaft_velocity_sp = P_angle( shaft_angle_sp - shaft_angle );
       // calculate the torque command
       current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity); // if voltage torque control
       // if torque controlled through voltage  
       if(torque_controller == TorqueControlType::voltage){
         // use voltage if phase-resistance not provided
         if(!_isset(phase_resistance))  voltage.q = current_sp;
         else  voltage.q = current_sp*phase_resistance;
         voltage.d = 0;
       }
       break;
     case MotionControlType::velocity:
       // velocity set point
       shaft_velocity_sp = target;
       // calculate the torque command
       current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity); // if current/foc_current torque control
       // if torque controlled through voltage control 
       if(torque_controller == TorqueControlType::voltage){
         // use voltage if phase-resistance not provided
         if(!_isset(phase_resistance))  voltage.q = current_sp;
         else  voltage.q = current_sp*phase_resistance;
         voltage.d = 0;
       }
       break;
     case MotionControlType::velocity_openloop:
       // velocity control in open loop
       shaft_velocity_sp = target;
       voltage.q = velocityOpenloop(shaft_velocity_sp); // returns the voltage that is set to the motor
       voltage.d = 0;
       break;
     case MotionControlType::angle_openloop:
       // angle control in open loop
       shaft_angle_sp = target;
       voltage.q = angleOpenloop(shaft_angle_sp); // returns the voltage that is set to the motor
       voltage.d = 0;
       break;
   }
 }
