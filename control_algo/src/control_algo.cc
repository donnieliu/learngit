/*==============================================================================
**
**  Name: car_egomotion.cpp
**
**  Descriptions:
**
**      Describe what this file does.
**
**  Date: 16-Nov-2020
**  Author: Fan Shengli
**
**  Copyright (c) 2020-2022 AICC Inc.
**
** PROPRIETARY RIGHTS of AICC Inc. are involved in the subject matter of
** this material.  All manufacturing, reproduction, use, and sales rights
** pertaining to this subject matter are governed by the license agreement.
** The recipient of this software implicitly accepts the terms of the license
**============================================================================*/
#include "control_algo.h"

#define C_INTERFACE

/**
 * @brief 加速度输出极值限制
 * lilang
 * @param long_acceleration ：纵向加速度 m/s2
 * @param vehiclespeed ：纵向车速 m/s
 */
void AccLimitFunc(float *long_acceleration, float vehiclespeed)
{
  float accmax,accmin ;
  float acc;
  
  acc = *long_acceleration;
  
  if(vehiclespeed < 5)//
  {
    accmax = 4.0f;
    accmin = -5.0f;
  }
  else if((vehiclespeed>=5)&&(vehiclespeed<=20))
  {
    accmax = -2.0f/15.0f*vehiclespeed+14.0f/3.0f;
    accmin = 0.1f*vehiclespeed-5.5f;
  }
  else
  {
    accmax = 2.0f;
    accmin = -3.5f;
  }
 
  *long_acceleration = fmax(fmin(acc,accmax),accmin);
}

/**
 * @brief AccJerkLimitFunc
 * 
 * @param accjerkmin m/s3
 * @param accjerkmax m/s3
 * @param vehiclespeed m/s
 */
void AccJerkLimitFunc(float *accjerkmin, float *accjerkmax, float vehiclespeed)
{
  if(vehiclespeed < 5)//
  {    
    *accjerkmin = -5.0f;
  }
  else if((vehiclespeed>=5)&&(vehiclespeed<=20))
  {
     *accjerkmin = vehiclespeed/6.0f-35.0f/6.0f;
  }
  else
  {
    *accjerkmin = -2.5f;    
  }
  *accjerkmax = 5.0f;
}

ControlAlgo::ControlAlgo(): received_time(0.0),
  current_time(0.0),
  angle_zero_shift_(2.0),
  start_flag(false)
{
  control_cmd_ = new VehicleCommandMsg();
  cipv_msg_ = new FrontVehicleMsg();
  can_msg_ = new VehicleReportMsg();
  odom_msg_ = new LocationMsg();
  trajectory_msg_ = new TrajectoryPlaningMsg();
  state_machine_msg_ = new VehicleHmiDisplay();
  acc_ctrl_msg_ = new HmiAccCtrlMsg ();


  odometry_received_ = false;
  trajectory_received_ = false;
  vehiclepar_received_ = false;
  cipv_received_ = false;//acc
  state_machine_received_ = false; // gaofeng 0303
  enable_acc_ = true;
}


int ControlAlgo::GetVehicleData(VehicleReportMsg *msg){
  std::lock_guard<std::mutex> lock(data_mutex_);
  memcpy(can_msg_, msg, sizeof(*msg));
  vehiclepar_received_ = true;
  return 0;
}

int  ControlAlgo::GetOdometryData(LocationMsg *msg){
  std::lock_guard<std::mutex> lock(data_mutex_);
  memcpy(odom_msg_, msg, sizeof(*msg));
  odometry_received_ = true;
  return 0;
}

int ControlAlgo::GetTrajectoryData(TrajectoryPlaningMsg *msg){
  std::lock_guard<std::mutex> lock(data_mutex_);
  memcpy(trajectory_msg_, msg, sizeof(*msg));
  trajectory_received_ = true;
  return 0;
}
//acc
int ControlAlgo::GetCipvData(FrontVehicleMsg *msg){
  std::lock_guard<std::mutex> lock(data_mutex_);
  memcpy(cipv_msg_, msg, sizeof(*msg));
  cipv_received_ = true;//acc
    return 0;
}

// gaofeng add
int ControlAlgo::GetStateMachineData(VehicleHmiDisplay *msg){
  std::lock_guard<std::mutex> lock(data_mutex_);
  memcpy(state_machine_msg_, msg, sizeof(*msg));
  state_machine_received_ = true;
    return 0;
}

int ControlAlgo::GetHmiCrlData(HmiAccCtrlMsg *msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    memcpy(acc_ctrl_msg_, msg, sizeof(*msg));
    return (SetAccSpdDistance());
}

int ControlAlgo::SetAccSpdDistance() {
    FDEBUG << "ControlAlgo::SetAccSpdDistance!!!!!!!!!!!!!!!!!!!!" << std::endl;
    FDEBUG << "acc_ctrl_msg_->on_off "<<acc_ctrl_msg_->on_off << std::endl;
    FDEBUG << "acc_ctrl_msg_->cruising_speed "<<acc_ctrl_msg_->cruising_speed << std::endl;
    FDEBUG << "acc_ctrl_msg_->timegap_toggle "<<acc_ctrl_msg_->timegap_toggle << std::endl;
    FDEBUG << "acc_ctrl_msg_->speed_plus_res "<<acc_ctrl_msg_->speed_plus_res << std::endl;
    FDEBUG << "acc_ctrl_msg_->speed_minus_set "<<acc_ctrl_msg_->speed_minus_set << std::endl;

     if(acc_ctrl_msg_->on_off == 1) 
     {
        time_gap_ = acc_ctrl_msg_->timegap_toggle;
        if (time_gap_ < 1.5) 
        {
          th_ = 1.0;
        } else if (time_gap_ < 2.5) 
        {
            th_ = 1.5;
        } else if (time_gap_ < 3.5) 
        {
            th_ = 1.9;
        } else if (time_gap_ < 4.5) 
        {
            th_ = 2.3;
        }
        if(state_machine_msg_->icc_active == 5)
        {
          th_ *= overtake_assist_th_discout_;
        }
      }
    return 0;
}

MsgHeader ControlAlgo::GetCurrentHeader() {
    MsgHeader current_header;
    auto now_clock = std::chrono::system_clock::now();
    uint64_t nanosec =
        std::chrono::duration_cast<std::chrono::nanoseconds>
        (now_clock.time_since_epoch()).count();
    current_header.data_stamp = nanosec;
    return current_header;
}

ControlAlgo::~ControlAlgo(){
  delete cipv_msg_;
  delete odom_msg_;
  delete can_msg_;
  delete trajectory_msg_;

  // gaofeng 0303
  delete control_cmd_;
  delete state_machine_msg_;

  delete acc_ctrl_msg_;
}
void ControlAlgo::Init(std::string *config_path) {

  LoadVehicleParameter(config_path);
  // LoadAccParameter(config_path);//acc
  aglo_seq_ = 0;
  this->purepursuit_ctr.Init(config_path);
  this->lqracc_ctr.Inital(config_path);
  eps_pid_.Init(config_path);//zhangzhuo
  time_gap_ = 3.0;
  FDEBUG << "ControlAlgo::Init over" << std::endl;
}
int ControlAlgo::ControlProc(){
  FDEBUG << "Control process......" << std::endl;
  FDEBUG << "Got Can_msg " << this->vehiclepar_received_
             << " Odom msg " << this->odometry_received_
             << " trajectory msg " << this->trajectory_received_
             << " cipv msg " << this->cipv_received_ << std::endl
             << "state machine msg" << this->state_machine_received_ << std::endl;
  if (this->vehiclepar_received_ && this->odometry_received_ && this->trajectory_received_ && this->cipv_received_)//
  {
    LocationMsg* tempary_odometry = new LocationMsg();
    TrajectoryPlaningMsg* tempary_trajectory = new TrajectoryPlaningMsg();
    VehicleReportMsg* tempary_vehicle = new VehicleReportMsg(); 
    FrontVehicleMsg* tempary_cipv = new FrontVehicleMsg();
    HmiAccCtrlMsg* acc_ctrl_msg = new HmiAccCtrlMsg ();

    memset(tempary_trajectory->trajectory_points, 0, sizeof(tempary_trajectory->trajectory_points));

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      memcpy(tempary_odometry, odom_msg_, sizeof(LocationMsg));
      memcpy(tempary_trajectory, trajectory_msg_, sizeof(TrajectoryPlaningMsg));
      memcpy(tempary_vehicle, can_msg_, sizeof(VehicleReportMsg));
      memcpy(tempary_cipv, cipv_msg_, sizeof(FrontVehicleMsg));
    }


    static int counter = 0;
    static double acc = -2;
    static double stee = 0.00001;
    int traj_point_size = tempary_trajectory->trajectory_points_size;

    if (traj_point_size < 1 ) {
        FERROR << " traj_point_size is less than 1" << std::endl;
        counter += 1;
        double acc_vehicle = -2;
        double steer_wheel = 0;
        if (counter > 2){
            GetControlCmdMessage(*tempary_cipv, acc_vehicle, steer_wheel);
        }else{
            GetControlCmdMessage(*tempary_cipv, acc, stee);
        }
    } else {
      // for(int i=0; i<traj_point_size; ++i) {
      //   std::cout<<" No ."<<i<<" , "<<tempary_trajectory->trajectory_points[i].way_point.x
      //                        <<" , "<<tempary_trajectory->trajectory_points[i].way_point.y<<std::endl;
      // }

      counter = 0;
      double delta(0.0), acceleration(0.0);
      this->purepursuit_ctr.LoadData(tempary_trajectory, tempary_odometry, tempary_vehicle);

      this->purepursuit_ctr.ControllerProc();

      this->lqracc_ctr.LoadData(tempary_trajectory, tempary_odometry, tempary_cipv, tempary_vehicle, th_);
      
      acceleration = this->lqracc_ctr.ControllerProc();

      double g_lowpass_gain_acc = 0.01;
      static double lowpass_acc = 0;
      lowpass_acc = g_lowpass_gain_acc * lowpass_acc + (1 - g_lowpass_gain_acc) * acceleration;
      acceleration = lowpass_acc;
      acc = acceleration;
      //acceleration = this->purepursuit_ctr.GetTargetAcceleration();
      delta = this->purepursuit_ctr.GetTargetSteerAngle();//deeg

      stee = delta;
      this->GetControlCmdMessage(*tempary_cipv, acceleration, delta);
    }
    delete tempary_odometry;
    delete tempary_trajectory;
    delete tempary_vehicle;
    delete tempary_cipv;
    delete acc_ctrl_msg;

    control_cmd_->chassis_command.header = GetCurrentHeader();
    if (aglo_seq_ > 500) {
      aglo_seq_ = 0;
    }
    control_cmd_->chassis_command.header.seq = ++aglo_seq_;

    return 0;
  } else {
    #ifdef LOG_ON
      FERROR << "Conditions is not fullfilled.Continue...........!" << std::endl;
    #endif
    return -1;
  }
}

// ZhangZhuo
// void ControlAlgo::LoadVehicleParameter(std::string *file_path)
// {
//   if (nullptr == file_path)
//   {
//     FERROR << "file_path pointer is empty!" << std::endl;
//     return;
//   } else {
//     FDEBUG << "conf file " << *file_path << std::endl;
//   }

//   cv::FileStorage fSettings;
//   fSettings.open(*file_path, cv::FileStorage::READ);
//   if(!fSettings.isOpened()){
//     FERROR << "failed to open file params.yaml " << std::endl;
//     return;
//   }
//   ptr_ego_params_->load_left_steer_anlge_max(fSettings["CtrlEgoParams.l_steer_max"]);
//   ptr_ego_params_->load_right_steer_anlge_max(fSettings["CtrlEgoParams.r_steer_max"]);
//   ptr_ego_params_->load_trans_ratio(fSettings["CtrlEgoParams.steer_ratio"]);
//   ptr_ego_params_->load_wheel_base(fSettings["CtrlEgoParams.wheel_base"]);
//   steer_step_max_ = fSettings["eps_limit.steer_step_max"];
//   overtake_assist_th_discout_ = fSettings["AccParams.overtake_assist_th_discout"];
//   driveoff_rel_dis_ = fSettings["AccParams.driveoff_rel_dis"];
//   driveoff_rel_vel_ = fSettings["AccParams.driveoff_rel_vel:"];
//   driveoff_enabled_vel_ = fSettings["AccParams.driveoff_enable_vel"];
//   driveoff_last_time_ = fSettings["AccParams.driveoff_last_time"];
//   driveoff_disabled_vel_ = fSettings["AccParams.driveoff_disable_vel"];
//   driveoff_standby_enable_vel_ = fSettings["AccParams.driveoff_standby_enable_vel"];
//   fSettings.release();
// }

void ControlAlgo::LoadVehicleParameter(std::string *file_path) {
  YAML::Node config = YAML::LoadFile(*file_path);

  overtake_assist_th_discout_  = config["StateMachineParams"]["overtake_assist_th_discout"].as<double>();
  driveoff_rel_dis_            = config["StateMachineParams"]["driveoff_rel_dis"].as<double>();
  driveoff_rel_vel_            = config["StateMachineParams"]["driveoff_rel_vel"].as<double>();
  driveoff_enabled_vel_        = config["StateMachineParams"]["driveoff_enable_vel"].as<double>();
  driveoff_last_time_          = config["StateMachineParams"]["driveoff_last_time"].as<double>();
  driveoff_disabled_vel_       = config["StateMachineParams"]["driveoff_disable_vel"].as<double>();
  driveoff_standby_enable_vel_ = config["StateMachineParams"]["driveoff_standby_enable_vel"].as<double>();
  FDEBUG << "yaml data : " << overtake_assist_th_discout_ << " , "
                           << driveoff_rel_dis_ << " , "
                           << driveoff_rel_vel_ << " , "
                           << driveoff_enabled_vel_ << " , "
                           << driveoff_last_time_ << " , "
                           << driveoff_disabled_vel_ << " , "
                           << driveoff_standby_enable_vel_
                           << std::endl;

}

void ControlAlgo::GetControlCmdMessage(const FrontVehicleMsg &cipv, double &acceleration, double &delta)
{
    static double pre_acc = 0;
    double aa = (acceleration - pre_acc);
    pre_acc = acceleration;
    control_cmd_->chassis_command.long_acceleration_cmd = acceleration;
    ENUM_GEAR gear_cmd = ENUM_GEAR::GEAR_DRIVE;//GEAR_INVALID;
    control_cmd_->chassis_command.gear_cmd = gear_cmd;
    control_cmd_->chassis_command.steer_angle_cmd = delta*DEG_TO_RAD;
    control_cmd_->chassis_command.steer_angle_rate_max_limit = 200*DEG_TO_RAD;//(this->stanley_ctr.ego_state.speed / this->stanley_ctr.ego_state.distance_between_wheel * std::tan(-delta) * M_PI / 180.0);
#ifdef ENABLE_SM
    StateMachine();
#endif

#ifdef ENABLE_DRIVEOFF
     control_cmd_->chassis_command.drive_off_enable = DriveOffEnable();
#endif

#ifdef ENABLE_DXTAROBJ_TRANSMIT
  control_cmd_->chassis_command.dx_tar_obj = DxTarObj();
#endif
}

// void ControlAlgo::StateMachine() {
//     FDEBUG << "state_machine_msg_->acc_active = " << state_machine_msg_->acc_active <<std::endl;
//     FDEBUG << "state_machine_msg_->icc_active = " << state_machine_msg_->icc_active <<std::endl;

//     if (state_machine_msg_->icc_active == 0 || state_machine_msg_->icc_active == 1 || state_machine_msg_->icc_active == 5)  {
//       if (!eps_pid_.is_reset()) {
//         eps_pid_.Reset();
//       }
//       control_cmd_->chassis_command.steer_angle_valid = 0;
//       control_cmd_->chassis_command.steer_angle_cmd = 0;
//       control_cmd_->chassis_command.steer_torque_valid = 0;
//       control_cmd_->chassis_command.steer_torque_cmd = 0;
//       if (state_machine_msg_->acc_active == 3) {  // acc active state
//           control_cmd_->chassis_command.acc_decto_stop_req = 0; // 停车请求
//           control_cmd_->chassis_command.acc_drive_off_req = 0;//驶离请求lilang

//           control_cmd_->chassis_command.stop_enable = 0;//lilang
//           control_cmd_->chassis_command.long_acceleration_valid = 1;//lilang
//           //control_cmd_->chassis_command.jerk_max = 4; // lilang
//           //control_cmd_->chassis_command.jerk_min = -4; // lilang
//           control_cmd_->chassis_command.comfort_boundary_low = -2;// lilang
//           control_cmd_->chassis_command.comfort_boundary_up  = 2;// lilang          
                          
//       } else if  ((state_machine_msg_->acc_active == 5) || 
//                   (state_machine_msg_->acc_active == 6)) {
//           control_cmd_->chassis_command.long_acceleration_cmd = 0;
//           control_cmd_->chassis_command.long_acceleration_valid = 0;
//           control_cmd_->chassis_command.acc_decto_stop_req = 1; // 停车请求
//           control_cmd_->chassis_command.stop_enable = 1;//lilang
//           control_cmd_->chassis_command.drive_off_enable = 0;//lilang
//           //control_cmd_->chassis_command.jerk_max = 4; // lilang
//           //control_cmd_->chassis_command.jerk_min = -4; // lilang
//           control_cmd_->chassis_command.comfort_boundary_low = -2;// lilang
//           control_cmd_->chassis_command.comfort_boundary_up  = 2;// lilang
          
//       } else if ((state_machine_msg_->acc_active == 0) ||
//                  (state_machine_msg_->acc_active == 1) ||
//                  (state_machine_msg_->acc_active == 2)  ||
//                  (state_machine_msg_->acc_active == 4) || // override
//                  (state_machine_msg_->acc_active ==  7) ||
//                  (state_machine_msg_->acc_active ==  8) ||
//                  (state_machine_msg_->acc_active ==  9) ||
//                  (state_machine_msg_->acc_active ==  10) ||
//                  (state_machine_msg_->acc_active ==  11) ||
//                  (state_machine_msg_->acc_active ==  12) ||
//                  (state_machine_msg_->acc_active ==  13) ||
//                  (state_machine_msg_->acc_active ==  14) ||
//                  (state_machine_msg_->acc_active ==  15)) {
//         control_cmd_->chassis_command.acc_decto_stop_req = 0; // 停车请求;
//         control_cmd_->chassis_command.drive_off_enable = 0;//lilang
//         control_cmd_->chassis_command.stop_enable = 0;//lilang
//         //control_cmd_->chassis_command.jerk_max = 1; // lilang
//         //control_cmd_->chassis_command.jerk_min = -1; // lilang
//         control_cmd_->chassis_command.comfort_boundary_low = -2;// lilang
//         control_cmd_->chassis_command.comfort_boundary_up  = 2;// lilang
//         control_cmd_->chassis_command.long_acceleration_cmd = 0;
//         control_cmd_->chassis_command.long_acceleration_valid = 0;

//       } else {
//         control_cmd_->chassis_command.acc_decto_stop_req = 0; // 停车请求;
//         control_cmd_->chassis_command.stop_enable = 0;//lilang
//         control_cmd_->chassis_command.drive_off_enable = 0;//lilang
//         //control_cmd_->chassis_command.jerk_max = 0; // lilang
//         //control_cmd_->chassis_command.jerk_min = 0; // lilang
//         control_cmd_->chassis_command.comfort_boundary_low = 0;// lilang
//         control_cmd_->chassis_command.comfort_boundary_up  = 0;// lilang
//         control_cmd_->chassis_command.long_acceleration_cmd = 0;
//         control_cmd_->chassis_command.long_acceleration_valid = 0;

//       }
//       double debug_temp1 = 66.0;
//       FDEBUG << "target_angle : "<< debug_temp1 << std::endl;
//       FDEBUG << "actual_angle : "<< debug_temp1 << std::endl;
//       FERROR<<" PID CTRL error : "<<1.5<<std::endl;
//       FERROR<<" PID CTRL speed : "<<1.5<<std::endl;
//       FDEBUG<<" PID CTRL p_term "<<1.5<<std::endl;
//       FDEBUG<<" PID CTRL i_term "<<1.5<<std::endl;
//       FDEBUG<<" PID CTRL d_term "<<1.5<<std::endl;
//       FDEBUG<<" PID CTRL ff_result "<<1.5<<std::endl;
//       FERROR<<" PID CTRL (pid result) : "<<1.5<<std::endl;

//     } else if ((state_machine_msg_->icc_active == 2) ||
//                (state_machine_msg_->icc_active == 3)) {
//         float taget_angle = 0.0;
//         float current_angle = 0.0;
//         float eps_coefficient = 1.0f;
//         control_cmd_->chassis_command.steer_torque_valid = 1;
//         control_cmd_->chassis_command.stop_enable = 0;
//         control_cmd_->chassis_command.long_acceleration_valid = 1;
//         //control_cmd_->chassis_command.jerk_max = 2.5; 
//         //control_cmd_->chassis_command.jerk_min = -2.5; 
//         control_cmd_->chassis_command.comfort_boundary_low = -2;
//         control_cmd_->chassis_command.comfort_boundary_up  = 2;
//         control_cmd_->chassis_command.drive_off_enable = 0;

//         taget_angle = control_cmd_->chassis_command.steer_angle_cmd / DEG_TO_RAD;
//         current_angle = can_msg_->chassis_report.steer_angle / DEG_TO_RAD;
        
//         if (eps_pid_.is_reset()) {
//           eps_pid_.CancelReset();
//         }
//         eps_pid_.PidCtrl(taget_angle, current_angle, can_msg_->chassis_report.velocity);
//         control_cmd_->chassis_command.steer_torque_cmd = eps_pid_.get_result();  
//     } else {
//       double debug_temp1 = 66.0;
//       FDEBUG << "target_angle : "<< debug_temp1 << std::endl;
//       FDEBUG << "actual_angle : "<< debug_temp1 << std::endl;
//       FERROR<<" PID CTRL target_angle : "<<1.5<<std::endl;
//       FERROR<<" PID CTRL actual_angle : "<<1.5<<std::endl;
//       FERROR<<" PID CTRL error : "<<1.5<<std::endl;
//       FERROR<<" PID CTRL speed : "<<1.5<<std::endl;
//       FDEBUG<<" PID CTRL p_term "<<1.5<<std::endl;
//       FDEBUG<<" PID CTRL i_term "<<1.5<<std::endl;
//       FDEBUG<<" PID CTRL d_term "<<1.5<<std::endl;
//       FDEBUG<<" PID CTRL ff_result "<<1.5<<std::endl;
//       FERROR<<" PID CTRL (pid result) : "<<1.5<<std::endl;
//     }
//     FDEBUG << "pid Reset : "<<eps_pid_.is_reset()<<std::endl;

//     AccLimitFunc(&control_cmd_->chassis_command.long_acceleration_cmd, 
//                  can_msg_->chassis_report.velocity);
        
//     AccJerkLimitFunc(&control_cmd_->chassis_command.jerk_min,
//                     &control_cmd_->chassis_command.jerk_max,
//                     can_msg_->chassis_report.velocity);

//     eps_pid_.epsTorqueLimitFunc(&control_cmd_->chassis_command.steer_torque_cmd);

//     FDEBUG << "long_acceleration_valid = " << control_cmd_->chassis_command.long_acceleration_valid <<std::endl;
//     FDEBUG << "long_acceleration_cmd = " << control_cmd_->chassis_command.long_acceleration_cmd <<std::endl;
//     FDEBUG << "steer_torque_valid = " << control_cmd_->chassis_command.steer_torque_valid <<std::endl;
//     FDEBUG << "steer_torque_cmd = " << control_cmd_->chassis_command.steer_torque_cmd <<std::endl;
// }
void ControlAlgo::StateMachine() 
{
  FDEBUG << "state_machine_msg_->acc_active = " << state_machine_msg_->acc_active <<std::endl;
  FDEBUG << "state_machine_msg_->icc_active = " << state_machine_msg_->icc_active <<std::endl;
  FDEBUG << "ldp_msg_.ldp_status = " << ldp_msg_.ldp_status <<std::endl;
  control_cmd_->chassis_command.steer_angle_valid = 0;
  control_cmd_->chassis_command.steer_torque_valid = 0;
  control_cmd_->chassis_command.long_acceleration_valid = 0;
  control_cmd_->chassis_command.steer_torque_cmd = 0;
  control_cmd_->chassis_command.acc_decto_stop_req = 0; // 停车请求
  control_cmd_->chassis_command.acc_drive_off_req = 0;//驶离请求lilang
  control_cmd_->chassis_command.stop_enable = 0;//lilang
  control_cmd_->chassis_command.comfort_boundary_low = 0;// lilang
  control_cmd_->chassis_command.comfort_boundary_up  = 0;// lilang
  control_cmd_->chassis_command.jerk_max = 0; // lilang
  control_cmd_->chassis_command.jerk_min = 0; // lilang
  if ((state_machine_msg_->icc_active == 0 || state_machine_msg_->icc_active == 1 || state_machine_msg_->icc_active == 5) && 
      (ldp_msg_.ldp_status == LDP_STATE_OFF || ldp_msg_.ldp_status == LDP_STATE_PASSIVE || ldp_msg_.ldp_status == LDP_STATE_STANDBY || ldp_msg_.ldp_status == LDP_STATE_ERROR))  
  {
    if (!eps_pid_.is_reset()) 
    {
      eps_pid_.Reset();
    }
    if (state_machine_msg_->acc_active == 3) 
    {  // acc active state
        control_cmd_->chassis_command.long_acceleration_valid = 1;//lilang
        control_cmd_->chassis_command.jerk_max = 4; // lilang
        control_cmd_->chassis_command.jerk_min = -4; // lilang
        control_cmd_->chassis_command.comfort_boundary_low = -2;// lilang
        control_cmd_->chassis_command.comfort_boundary_up  = 2;// lilang                            
    } 
    else if (state_machine_msg_->acc_active == 5 || state_machine_msg_->acc_active == 6) 
    {
        control_cmd_->chassis_command.long_acceleration_cmd = 0;
        control_cmd_->chassis_command.acc_decto_stop_req = 1; // 停车请求
        control_cmd_->chassis_command.stop_enable = 1;//lilang 
    } 
    else 
    {
      control_cmd_->chassis_command.long_acceleration_cmd = 0;
    }
    double debug_temp1 = 66.0;
    FDEBUG << "target_angle : "<< debug_temp1 << std::endl;
    FDEBUG << "actual_angle : "<< debug_temp1 << std::endl;
    FERROR<<" PID CTRL error : "<<1.5<<std::endl;
    FERROR<<" PID CTRL speed : "<<1.5<<std::endl;
    FDEBUG<<" PID CTRL p_term "<<1.5<<std::endl;
    FDEBUG<<" PID CTRL i_term "<<1.5<<std::endl;
    FDEBUG<<" PID CTRL d_term "<<1.5<<std::endl;
    FDEBUG<<" PID CTRL ff_result "<<1.5<<std::endl;
    FERROR<<" PID CTRL (pid result) : "<<1.5<<std::endl;
  } 
  else if (state_machine_msg_->icc_active == 2 || state_machine_msg_->icc_active == 3 || ldp_msg_.ldp_status == LDP_STATE_ACTIVE) 
  {
    float taget_angle = 0.0;
    float current_angle = 0.0;
    float eps_coefficient = 1.0f;
    if(state_machine_msg_->acc_active == ACC_STATUS_ACTIVE)
    {
      control_cmd_->chassis_command.long_acceleration_valid = 1;
      control_cmd_->chassis_command.jerk_max = 4; 
      control_cmd_->chassis_command.jerk_min = -4; 
      control_cmd_->chassis_command.comfort_boundary_low = -2;
      control_cmd_->chassis_command.comfort_boundary_up  = 2;
    }
    else
    {
      control_cmd_->chassis_command.long_acceleration_cmd = 0;
    }
    control_cmd_->chassis_command.steer_torque_valid = 1;
    taget_angle = control_cmd_->chassis_command.steer_angle_cmd / DEG_TO_RAD;
    current_angle = can_msg_->chassis_report.steer_angle / DEG_TO_RAD;
    if (eps_pid_.is_reset()) {
      eps_pid_.CancelReset();
    }
    eps_pid_.PidCtrl(taget_angle, current_angle, can_msg_->chassis_report.velocity);
    control_cmd_->chassis_command.steer_torque_cmd = eps_pid_.get_result();  
  } 
  else 
  {
    double debug_temp1 = 66.0;
    FDEBUG << "target_angle : "<< debug_temp1 << std::endl;
    FDEBUG << "actual_angle : "<< debug_temp1 << std::endl;
    FERROR<<" PID CTRL target_angle : "<<1.5<<std::endl;
    FERROR<<" PID CTRL actual_angle : "<<1.5<<std::endl;
    FERROR<<" PID CTRL error : "<<1.5<<std::endl;
    FERROR<<" PID CTRL speed : "<<1.5<<std::endl;
    FDEBUG<<" PID CTRL p_term "<<1.5<<std::endl;
    FDEBUG<<" PID CTRL i_term "<<1.5<<std::endl;
    FDEBUG<<" PID CTRL d_term "<<1.5<<std::endl;
    FDEBUG<<" PID CTRL ff_result "<<1.5<<std::endl;
    FERROR<<" PID CTRL (pid result) : "<<1.5<<std::endl;
  }
  FDEBUG << "pid Reset : "<<eps_pid_.is_reset()<<std::endl;
  AccLimitFunc(&control_cmd_->chassis_command.long_acceleration_cmd, 
                can_msg_->chassis_report.velocity);     
  AccJerkLimitFunc(&control_cmd_->chassis_command.jerk_min,
                  &control_cmd_->chassis_command.jerk_max,
                  can_msg_->chassis_report.velocity);
  eps_pid_.epsTorqueLimitFunc(&control_cmd_->chassis_command.steer_torque_cmd);
  FDEBUG << "long_acceleration_valid = " << control_cmd_->chassis_command.long_acceleration_valid <<std::endl;
  FDEBUG << "long_acceleration_cmd = " << control_cmd_->chassis_command.long_acceleration_cmd <<std::endl;
  FDEBUG << "steer_torque_valid = " << control_cmd_->chassis_command.steer_torque_valid <<std::endl;
  FDEBUG << "steer_torque_cmd = " << control_cmd_->chassis_command.steer_torque_cmd <<std::endl;
}
bool ControlAlgo::DriveOffEnable()
{
  static bool drive_off_enable = false;
  static bool drive_off_timeout_flag = false;
  int64_t time_now = std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::system_clock::now().time_since_epoch()).count();
  double time_now_s = static_cast<double>(time_now)/1000.0;
  static double drive_off_start_time = time_now_s;
  static int8_t last_driveoff_state = state_machine_msg_->acc_active;
  static int8_t last_driveoff_period_state = state_machine_msg_->acc_active;
  //Temprarymode-Acc/STop_mode-Acc
  if (state_machine_msg_->acc_active == 5 || (last_driveoff_state == 5 && state_machine_msg_->acc_active == 3))
  {
    if(cipv_msg_->is_valid)
    {
      if((cipv_msg_->obj_velocity > driveoff_rel_vel_ || cipv_msg_->distance_to_ego_vehicle > driveoff_rel_dis_) && !drive_off_timeout_flag)
      {
        drive_off_enable = true;
      }
    }
  }
  else if(state_machine_msg_->acc_active == 6 || (last_driveoff_state == 6 && state_machine_msg_->acc_active == 3))
  {
    
    if((((cipv_msg_->obj_velocity > driveoff_rel_vel_ || cipv_msg_->obj_velocity > driveoff_rel_vel_) && cipv_msg_->is_valid) || can_msg_->chassis_report.velocity > driveoff_enabled_vel_ ||  !cipv_msg_->is_valid) && !drive_off_timeout_flag)
    {
      drive_off_enable = true;
    }
  }
  else if(state_machine_msg_->acc_active ==3 && last_driveoff_state == 2)
  {
    if(can_msg_->chassis_report.velocity < driveoff_standby_enable_vel_ && control_cmd_->chassis_command.long_acceleration_cmd > 0 && !drive_off_timeout_flag)
    {
      drive_off_enable = true;
    }
  }
  //drive_off_enable  is disabled  when current velocity is over 9km/h and ACC is lost.
  if(can_msg_->chassis_report.velocity > driveoff_disabled_vel_/3.6 || (state_machine_msg_->acc_active !=3 && last_driveoff_period_state == 3))
  {
    drive_off_enable = false;
    drive_off_timeout_flag = false;    
  }

  //time of drive_off_enable does not last for driveoff_last_time_
  if(drive_off_enable && !drive_off_timeout_flag)
  {
    drive_off_start_time = time_now_s;
    drive_off_timeout_flag = true;
  }
  if(drive_off_timeout_flag)
  {
    if((time_now_s - drive_off_start_time) > driveoff_last_time_)
    {
      drive_off_enable = false;
      drive_off_timeout_flag = false;
    }
  }
  if(last_driveoff_period_state != state_machine_msg_->acc_active)
  {
      last_driveoff_state = last_driveoff_period_state;
  }
  last_driveoff_period_state = state_machine_msg_->acc_active;
  FDEBUG << "drive_off_enable = " << drive_off_enable <<std::endl;
  FDEBUG << "last_driveoff_state = " << last_driveoff_state <<std::endl;
  FDEBUG << "drive_off_timeout_flag = " << drive_off_timeout_flag <<std::endl;
  return drive_off_enable;
}
int8_t ControlAlgo::DxTarObj()
{
  int8_t th_grade = 3;
  double th = 1.9;
  if(state_machine_msg_->acc_active ==2 || state_machine_msg_->acc_active ==3 || state_machine_msg_->acc_active ==5 || state_machine_msg_->acc_active ==6)
  {
    if(cipv_msg_->is_valid)
    {
        if(can_msg_->chassis_report.velocity < 20.0/3.6)
        {
          if(cipv_msg_->distance_to_ego_vehicle < 3)
          {
              th_grade = 1;
          }
          else if(cipv_msg_->distance_to_ego_vehicle < 4)
          {
            th_grade = 2;
          }
          else if(cipv_msg_->distance_to_ego_vehicle < 5)
          {
            th_grade = 3;
          }
          else 
          {
            th_grade = 4;
          }
        }
        else
        {
          if(cipv_msg_->distance_to_ego_vehicle < 5)
          {
            th_grade = 1;
          }
          else
          {
              th = cipv_msg_->distance_to_ego_vehicle/can_msg_->chassis_report.velocity;
              if(th < 1.5)
              {
                th_grade = 1;
              }
              else if(th < 1.9)
              {
                th_grade = 2;
              }
              else if(th < 2.3)
              {
                th_grade = 3;
              }
              else
              {
                th_grade = 4;
              }
          }
        }
    }
    else
    {
      th_grade = 4;
    }
    
  }
  FDEBUG << "th_grade = " << th_grade <<std::endl;
}
bool ControlAlgo::AccMinimumBraking()
{
  static bool acc_minimum_braking_enable;
  static int8_t last_acc_state = state_machine_msg_->acc_active;
  static double last_cruising_speed = acc_ctrl_msg_->cruising_speed ;
  if(can_msg_->chassis_report.velocity - acc_ctrl_msg_->cruising_speed > 1.8/3.6)
  {
    if((last_acc_state == 2 && state_machine_msg_->acc_active == 3) || (last_acc_state == 4 && state_machine_msg_->acc_active == 3) )
    {
      acc_minimum_braking_enable = true;
    }
  }

  if(acc_ctrl_msg_->cruising_speed != last_cruising_speed || (can_msg_->chassis_report.velocity - acc_ctrl_msg_->cruising_speed > 1.8/3.6))
  {
    acc_minimum_braking_enable = true;
  }  
  last_acc_state = state_machine_msg_->acc_active;
  last_cruising_speed = acc_ctrl_msg_->cruising_speed;
  FDEBUG << "acc_minimum_braking_enable = " << acc_minimum_braking_enable <<std::endl;
  return acc_minimum_braking_enable;
}
bool ControlAlgo::ACCTakeoverRequest()
{
  //acc condition?
  if(state_machine_msg_->acc_active == 3)
  {
    if(cipv_msg_->is_valid )
    {
      //safe_dec_require
      double cipv_info_v = cipv_msg_->obj_velocity + can_msg_->chassis_report.velocity;
      double safe_require_dec = (cipv_info_v * cipv_info_v - can_msg_->chassis_report.velocity * can_msg_->chassis_report.velocity)/2.0 * cipv_msg_->distance_to_ego_vehicle;
      double dec_require_t = (cipv_info_v - can_msg_->chassis_report.velocity)/safe_require_dec;
      double AccelFactor = 0.8;
      if(dec_require_t <= 1.0) 
      {
        AccelFactor = 0.5;
      } 
      else if(dec_require_t <= 2.0)
      {
        AccelFactor = 0.5 + (0.8 - 0.5)/(2.0-1.0)*(dec_require_t - 1.0);
      }
      else if(dec_require_t <= 8.0)
      {
        AccelFactor = 0.8 + (1.1 - 0.8)/(8.0-2.0)*(dec_require_t - 2.0);
      }
      else if(dec_require_t <= 15.0)
      {
        AccelFactor = 1.1 + (10.0 - 1.1)/(15.0-8.0)*(dec_require_t - 8);
      }
      else
      {
        AccelFactor = 10;
      }
      FDEBUG << "safe_require_dec= " << safe_require_dec << std::endl;
      FDEBUG << "AccelFactor= " << AccelFactor << std::endl;
      if(safe_require_dec < AccelFactor*(-3))
      {
        return true;
      }
      //relative_dis_require
      double TauTOR = 0.55;
      if(cipv_msg_->obj_velocity < -13.0)
      {
          TauTOR = 0.55;
      }
      else if(cipv_msg_->obj_velocity < 2.0)
      {
        TauTOR = 0.55 - (0.55- 0.15)/(2.0-(-13.0))*(cipv_msg_->obj_velocity +13.0);
      }
      else 
      {
        TauTOR = 0.15;
      }
      if(cipv_msg_->distance_to_ego_vehicle < can_msg_->chassis_report.velocity * TauTOR)
      {
        FDEBUG << "TauTOR1= " << TauTOR << std::endl;
        return true;
      }
      FDEBUG << "TauTOR2= " << TauTOR << std::endl;
      return false;
    }
    else
    {
      return false;
    } 
  }
  else
  {
    return false;
  } 
}

