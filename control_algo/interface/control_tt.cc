#include <mutex>
#include <thread>
#include <yaml-cpp/yaml.h>

#include "api_func_types.h"
#include "control_algo.h"
#include "interface.h"
#include "location.h"
#include "planning.h"
#include "situation.h"

std::thread timer_thd;

const ApiFuncList* os_api;

std::string config_file_path;

ControlAlgo* ptr_control_algo;

int HMI_ENABLE_AAUTO_DRIVE = 0;
int count = 0;

int unique_id_;
int pre_unique_id_;
std::mutex pub_mutex;

int process_vehicle(void* msg) {
  auto xbuf = (aicc_xbuf_t*)msg;
  if (xbuf->data_length != sizeof(VehicleReportMsg)) {
    std::cout << "recv VehicleReportMsg data size err" << std::endl;
    std::cout << "xbuf->data_length = " << xbuf->data_length <<" sizeof(VehicleReportMsg) = " <<  sizeof(VehicleReportMsg) << std::endl;

    return 1;
  }
  auto p_can_msg = (VehicleReportMsg*)aicc_buf_get_data(xbuf);

 // std::cout<<"process_vehicle !!!!!!!!!!!!!!!"<<std::endl;
  return (ptr_control_algo->GetVehicleData(p_can_msg));
}

int process_odometry(void* msg) {
  auto xbuf = (aicc_xbuf_t*)msg;
  if (xbuf->data_length != sizeof(LocationMsg)) {
    std::cout << "recv LocationMsg data size err" << std::endl;
    return 1;
  }
  auto p_location_msg = (LocationMsg*)aicc_buf_get_data(xbuf);

 //  std::cout<<"process_odometry !!!!!!!!!!!!!!!"<<std::endl;
  return (ptr_control_algo->GetOdometryData(p_location_msg));
}

int process_trajectory(void* msg) {
  auto xbuf = (aicc_xbuf_t*)msg;
  if (xbuf->data_length != sizeof(TrajectoryPlaningMsg)) {
    std::cout << "recv TrajectoryPlaningMsg data size err" << std::endl;
    return 1;
  }
  auto p_traj_msg = (TrajectoryPlaningMsg*)aicc_buf_get_data(xbuf);

  return (ptr_control_algo->GetTrajectoryData(p_traj_msg));
}

int process_Cipv(void* msg) {
  auto xbuf = (aicc_xbuf_t*)msg;
  if (xbuf->data_length != sizeof(FrontVehicleMsg)) {
    std::cout << "recv FrontVehicleMsg data size err" << std::endl;
    return 1;
  }
  auto p_cipv_msg = (FrontVehicleMsg*)aicc_buf_get_data(xbuf);

  return (ptr_control_algo->GetCipvData(p_cipv_msg));
}

int process_hmi(void* msg) {
  // std::cout << "in process_hmi" << std::endl;
  // auto pbmsg = std::dynamic_pointer_cast<HmiDisplayMsg>(msg);
  // if (pbmsg->acc_active() == aicc::icvos::VehicleHmiDisplay::Active) {
  //   ICVOS_INFO<<"ACC is Enabled !! ";
  //   enable_acc(true);
  // } else {
  //   ICVOS_INFO<<"ACC is Disabled !! ";
  //   enable_acc(false);
  // }
  //   enable_acc(true);
   return 1;
}

int process_set_speed(void* msg) {
  auto xbuf = (aicc_xbuf_t*)msg;
  if (xbuf->data_length != sizeof(HmiAccCtrlMsg)){
    std::cout << "recv HmiAccCtrlMsg data size err" << std::endl;
    return 1;
  }
  auto p_speed_msg = (HmiAccCtrlMsg*)aicc_buf_get_data(xbuf);
  ptr_control_algo->GetHmiCrlData(p_speed_msg);
  return 0;
}


  // gaofeng
   // 状态机结构体
int process_state_machine(void* msg/*VehicleHmiDisplay *msg*/)  
{
      auto xbuf = (aicc_xbuf_t*)msg;
      if (xbuf->data_length != sizeof(VehicleHmiDisplay)) {
      std::cout << "recv VehicleHmiDisplay data size err" << std::endl;
      return 1;
      }
      auto p_state_machine_msg = (VehicleHmiDisplay*)aicc_buf_get_data(xbuf);

      return (ptr_control_algo->GetStateMachineData(p_state_machine_msg));
}


extern "C" int unload(void) {
  delete ptr_control_algo;
  if (ptr_control_algo != NULL) {
    ptr_control_algo = NULL;
  }
  return 0;
}

void control_timer_callback() {
  int ctrl_ok = ptr_control_algo->ControlProc();
  if (ctrl_ok == 0) {
    ++unique_id_;
  }
  std::lock_guard<std::mutex> lock(pub_mutex);
  if (pre_unique_id_ != unique_id_) {
    auto xbuf_command = aicc_buf_get(sizeof(VehicleCommandMsg), "VehicleCommandMsg");
    auto out_message = (VehicleCommandMsg*)aicc_buf_get_data(xbuf_command);
    auto tp_ptr = ptr_control_algo->ctrl_output();
    memcpy(out_message, tp_ptr, sizeof(VehicleCommandMsg));
    os_api->publish(os_api->node, "VehicleCommandMsg", xbuf_command);
    aicc_buf_free((aicc_xbuf_t*)xbuf_command);
    pre_unique_id_ = unique_id_;
  }
}

extern "C" int init(const char* config_path, const ApiFuncList* api) {
  google::InitGoogleLogging("ControlAlgo");
  FLAGS_log_dir = "/home/root/logs";
  FLAGS_logtostderr = 0;
  FLAGS_alsologtostderr = 1;
  FLAGS_minloglevel = 0;

  std::string config_path_s = config_path;

  YAML::Node config = YAML::LoadFile(config_path_s);
  double node_loop_s = config["VehicleParams"]["l_steer_max"].as<double>();
  double node_loop_ms = node_loop_s * 1000;
  
  os_api = api;

  os_api->register_sub(os_api->node, "/ICVOS/EX/Vehicle/Report",
                       process_vehicle);
  os_api->register_sub(os_api->node, "/ICVOS/Location/Odometry",
                       process_odometry);
  os_api->register_sub(os_api->node, "/ICVOS/Plan/Trajectory",
                       process_trajectory);
  os_api->register_sub(os_api->node, "/ICVOS/FN/Plan/LeadVehicle",
                       process_Cipv);
  os_api->register_timer(os_api->node, node_loop_ms, control_timer_callback);

  // gaofeng
  os_api->register_sub(os_api->node, "/ICVOS/Sys/Ctrl/HmiDisplay",
                       process_state_machine);
  os_api->register_sub(os_api->node, "/ICVOS/Sys/Ctrl/HmiAcc", process_set_speed);

  
  ptr_control_algo = new ControlAlgo();

  ptr_control_algo->Init(&config_path_s);

  std::cout << "algo init done" << std::endl;
  return 0;
}
