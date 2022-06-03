/*==============================================================================
**
**  Name: car_egomotion.h
**
**  Descriptions:
**
**      Describe what this file does.
**
**  Date: 02-Nov-2020
**  Author: Fan shengli
**
**  Copyright (c) 2020-2022 AICC Inc.
**
** PROPRIETARY RIGHTS of AICC Inc. are involved in the subject matter of
** this material.  All manufacturing, reproduction, use, and sales rights
** pertaining to this subject matter are governed by the license agreement.
** The recipient of this software implicitly accepts the terms of the license
**============================================================================*/

#ifndef CONTROL_ALGORITHM_H_
#define CONTROL_ALGORITHM_H_
// #define SIM_DEBUG
// #include <ados_localization_msgs/msg/can_information.hpp>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <unordered_map>
#include <chrono>
#include <cstdlib>
#include <math.h>
#include <time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>

// #include "stanley/stanley_controller.h"
#include "purepursuit/purepursuit.h"
// #include "vehicle_params/vehicle_params.h"
#include "lqr_acc/lqr_acc.h"//acc
// #include "acc_params/acc_params.h"//acc
#include "eps_pid.h"//zhangzhuo

#define SIM_DEBUG 
#define ENABLE_SM

class ControlAlgo {
public:
  ControlAlgo();
  ~ControlAlgo();
  ControlAlgo(const ControlAlgo &) = delete;
  ControlAlgo &operator=(const ControlAlgo &) = delete;
  ControlAlgo(ControlAlgo &&) = delete;
  ControlAlgo &operator=(ControlAlgo &&) = delete;

  int GetVehicleData(VehicleReportMsg *msg);
  int GetOdometryData(LocationMsg *msg);
  int GetTrajectoryData(TrajectoryPlaningMsg *msg);
  int GetCipvData(FrontVehicleMsg *pbmsg);
  int GetHmiCrlData(HmiAccCtrlMsg *msg);

// gaofeng  0303
//====================================
  int GetStateMachineData(VehicleHmiDisplay *msg);
  VehicleHmiDisplay  *state_machine_msg_;
  VehicleCommandMsg *control_cmd_;
   VehicleReportMsg *can_msg_;  // gaofeng
   HmiAccCtrlMsg* acc_ctrl_msg_;
//=====================================
  void Init(std::string *config_path);
  int ControlProc();
  void StateMachine();
  VehicleCommandMsg* ctrl_output() {
    return control_cmd_;
  }
  void set_auto_drive(const int& enable) {
    HMI_ENABLE_AAUTO_DRIVE = enable;
  }

  void enable_acc(const bool& enable) {
    enable_acc_ = enable;
  }
private:
  void GetControlCmdMessage(const FrontVehicleMsg &cipv, double &acceleration, double &delta);
  MsgHeader GetCurrentHeader();
  void LoadVehicleParameter(std::string *file_path);
  int SetAccSpdDistance();
  bool DriveOffEnable();
  int8_t DxTarObj();
  bool AccMinimumBraking();
  bool ACCTakeoverRequest();
private:
  std::mutex data_mutex_;

  FrontVehicleMsg *cipv_msg_;
 //  VehicleReportMsg *can_msg_;  // gaofeng移动到public
  LocationMsg* odom_msg_;
  TrajectoryPlaningMsg* trajectory_msg_;
  LDPSmPubMsg ldp_msg_;
  
  double received_time;
  double current_time;
  double angle_zero_shift_;
  bool start_flag;
  double dt;

  // StanleyController stanley_ctr;
  PurePursuit purepursuit_ctr;
  LqrAcc lqracc_ctr;//acc
  // std::ofstream  fwrite;
  std::string file_fullname;

  bool odometry_received_;
  bool trajectory_received_;
  bool vehiclepar_received_;
  bool cipv_received_;//acc
  // gaofeng add 0303
  bool state_machine_received_;

  std::mutex mutex_;
  int HMI_ENABLE_AAUTO_DRIVE;
  int aglo_seq_;
  bool enable_acc_;
  float time_gap_;

  eps_ctrl::PID eps_pid_;//zhangzhuo
  double overtake_assist_th_discout_;
  double th_;
  double driveoff_rel_dis_;
  double driveoff_rel_vel_;
  double driveoff_enabled_vel_;
  double driveoff_last_time_;
  double driveoff_disabled_vel_;
  double driveoff_standby_enable_vel_;
};
#endif // CONTROL_ALGORITHM_H_
