#include "lqr_acc.h"
//#include "rclcpp/rclcpp.hpp"

LqrAcc::LqrAcc()
{
    // v_set_ = 0.0;
	// ts_ = 0.0;
	th_ = 1.9;
	// d_offset_ = 0.0;
	//state_size_ = 0.0;
	// tolerance_ = 0.0;
	// max_num_iteration_ = 0.0;
    ua_lbound_ = -3.0;
    ua_ubound_ = 2.0;
    loop_rate_ = 0.02;
    acc_pid_prev_i_term_ = 0.0;
    prev_speed_error_ = 0.0;

}

LqrAcc::~LqrAcc() {}

// void LqrAcc::Inital(double v_set, double ts, double th, double d_offset, double state_size_, double tolerance, double max_num_iteration){
void LqrAcc::Inital(std::string *file_path) {
  LoadConf(file_path);
  MatrixInit();
  FDEBUG << "MatrixInit ok " << std::endl;
}

int LqrAcc::LoadConf(std::string *file_path) {
  YAML::Node config = YAML::LoadFile(*file_path);

  acc_params_.load_ts(config["LQRsolvor"]["ts"].as<double>());
  acc_params_.load_state_size(config["LQRsolvor"]["state_size"].as<double>());
  acc_params_.load_tolerance(config["LQRsolvor"]["tolerance"].as<double>());
  acc_params_.load_max_num_iteration(config["LQRsolvor"]["num_iteration"].as<double>());

  acc_params_.load_acc_kp(config["SpeedPIDCtrl"]["kp"].as<double>());
  acc_params_.load_acc_ki(config["SpeedPIDCtrl"]["ki"].as<double>());
  acc_params_.load_acc_kd(config["SpeedPIDCtrl"]["kd"].as<double>());
  acc_params_.load_acc_itrem_max(config["SpeedPIDCtrl"]["iterm_max"].as<double>());
  acc_params_.load_acc_itrem_min(config["SpeedPIDCtrl"]["iterm_min"].as<double>());

  acc_params_.load_d_offset(config["ACCParams"]["d_offset"].as<double>());

  acc_params_.load_q(config["DisLQRCtrl"]["q1"].as<double>(),
                     config["DisLQRCtrl"]["q2"].as<double>(),
                     config["DisLQRCtrl"]["q3"].as<double>(),
                     config["DisLQRCtrl"]["q4"].as<double>());
  acc_params_.load_r(config["DisLQRCtrl"]["r1"].as<double>());
  loop_rate_ = config["NodeSet"]["ctrl_loop"].as<double>();

  FDEBUG << "LQR config " << acc_params_.get_ts() << " , "
                          << acc_params_.get_state_size() << " , "
                          << acc_params_.get_tolerance() << " , "
                          << acc_params_.get_max_num_iteration() << " , "
                          << acc_params_.get_acc_kp() << " , "
                          << acc_params_.get_acc_ki() << " , "
                          << acc_params_.get_acc_kd() << " , "
                          << acc_params_.get_acc_iterm_min() << " , "
                          << acc_params_.get_acc_iterm_max() << " , "
                          << acc_params_.get_d_offset() << " , "
                          << acc_params_.get_q1() << " , "
                          << acc_params_.get_q2() << " , "
                          << acc_params_.get_q3() << " , "
                          << acc_params_.get_q4() << " , "
                          << acc_params_.get_r1() << " , "
                          << loop_rate_ << " , "
                          << std::endl;
    return 0;
}

void LqrAcc::MatrixInit(){
    // Matrix init operatinons
    const int matrix_size = acc_params_.get_state_size();
   // std::cout << "matrixinit" << "  " << "state_size" << matrix_size << std::endl;
    // init vehicle states matrix
    matrix_a_ = Matrix::Zero(matrix_size, matrix_size);
    matrix_ad_ = Matrix::Zero(matrix_size, matrix_size);
    matrix_a_(0, 0) = 0.0;
    matrix_a_(0, 1) = -1.0;
    matrix_a_(1, 0) = 0.0;
    matrix_a_(1, 1) = 0.0;
    Matrix matrix_i = Matrix::Identity(matrix_a_.cols(), matrix_a_.cols());
    matrix_ad_ = (matrix_i - acc_params_.get_ts() * 0.5 * matrix_a_).inverse() * (matrix_i + acc_params_.get_ts() * 0.5 * matrix_a_);

    // init control matrix
    matrix_b_ = Matrix::Zero(matrix_size, 1);
    matrix_bd_ = Matrix::Zero(matrix_size, 1);
    matrix_b_(0, 0) = 0.0;
    matrix_b_(1, 0) = -1.0;
    matrix_bd_ = matrix_b_ * acc_params_.get_ts();

    matrix_state_ = Matrix::Zero(matrix_size, 1);
    matrix_k_ = Matrix::Zero(1, matrix_size);
    matrix_q_ = Matrix::Zero(matrix_size, matrix_size);
    matrix_r_ = Matrix::Identity(1,1);

    matrix_q_(0,0) = acc_params_.get_q1();
    matrix_q_(0,1) = acc_params_.get_q2();
    matrix_q_(1,0) = acc_params_.get_q3();
    matrix_q_(1,1) = acc_params_.get_q4();

    matrix_r_(0,0) = acc_params_.get_r1();
    //std::cout << "matrixinit end" << std::endl;
}

void LqrAcc::LoadData(const TrajectoryPlaningMsg* trajectory,
    const LocationMsg *ego_pose, const FrontVehicleMsg* cipv, const VehicleReportMsg* ego_can, float th) {
    plan_trajectory_.clear();
    int tra_size = trajectory->trajectory_points_size;//sizeof(trajectory->trajectory_points) / sizeof(AiccTrajectoryPoint);
	for (size_t i=0; i<tra_size; ++i) {
		auto tp_p = trajectory->trajectory_points[i];
		TrajectoryPoint_acc tp_tra_p(tp_p.way_point.x, tp_p.way_point.y,
            tp_p.way_point.theta, tp_p.way_point.s, tp_p.speed);
		plan_trajectory_.push_back(tp_tra_p);
	}
	double ego_x = 0.0;//ego_pose->pose.position.x;
	double ego_y = 0.0;//ego_pose->pose.position.y;
	double ego_yaw = 0.0;
                 //GetYaw(ego_pose->pose.orientation.qx,
		                    //ego_pose->pose.orientation.qy,
		                    //ego_pose->pose.orientation.qz,
		                    //ego_pose->pose.orientation.qw); //deeg

	ego_state_.yaw_rate = ego_can->chassis_report.ego_motion.yaw_rate / DEG_TO_RAD; //deeg/s
	double ego_speed = ego_can->chassis_report.velocity;//for simulation
	ego_state_.steer_angle = ego_can->chassis_report.steer.steer_angle / DEG_TO_RAD;//deeg/s
	ego_state_.load_pose_value(ego_x, ego_y, ego_yaw, ego_speed);

    cipv_info_.is_valid = cipv->is_valid;
    cipv_info_.dis = cipv->distance_to_ego_vehicle;
    // Hu 2022.1.13
    // cipv_info_.v = cipv->obj_velocity;
    cipv_info_.v = cipv->obj_velocity + ego_can->chassis_report.velocity;

    can_info_.ego_v = ego_can->chassis_report.velocity;
    th_ = th;
    // if (time_gap < 1.5) {
    //   th_ = 1.0;
    // } else if (time_gap < 2.5) {
    //   th_ = 1.5;
    // } else if (time_gap < 3.5) {
    //   th_ = 1.9;
    // } else if (time_gap < 4.5) {
    //   th_ = 2.3;
    // }

    // std::cout<<"############## LQR Load Data #############"<<std::endl;
    FDEBUG << " ego info "<<std::endl;
    FDEBUG << "             ego_v         :  "<<ego_state_.speed<<std::endl;
    FDEBUG << "             location  :  "<<ego_state_.x<<" , y : "<<ego_state_.y<<" , yaw : "<<ego_state_.yaw<<std::endl;
    FDEBUG << " cipv info "<<std::endl;
    FDEBUG << "             is valid ?   "<<cipv_info_.is_valid<<std::endl;
    FDEBUG << "             cipv_v          : "<<cipv_info_.v<<std::endl;
    FDEBUG << "             dis to ego : "<<cipv_info_.dis<<std::endl;
    FDEBUG << " time_gap is "<<th_<<std::endl;
    // std::cout<<">>>traj info "<<std::endl;
    // std::cout<<"             traj size  : "<<plan_trajectory_.size()<<std::endl;
    // int ii = plan_trajectory_.size();
    // if (ii > 10){
    //     ii = 10;
    // }
    // for(int i=0; i<ii; ++i){
    //     std::cout<<"         No."<<i
    //              <<" , x : "<<plan_trajectory_.at(i).x
    //              <<" , y : "<<plan_trajectory_.at(i).y
    //              <<" , s : "<<plan_trajectory_.at(i).s
    //              <<" , v : "<<plan_trajectory_.at(i).v<<std::endl;
    // }

}

void LqrAcc::UpdateState(const double &d_error, const double &v_error){
    matrix_state_(0, 0) = d_error;
    matrix_state_(1, 0) = v_error;

    // matrix_q_(0,0) = 1;
    // matrix_q_(0,1) = 0;
    // matrix_q_(1,0) = 0;
    // matrix_q_(1,1) = 4;

    // matrix_r_(0,0) = 9;//5 9 15
    //std::cout << "state, q, r" << std::endl;
}

double LqrAcc::ControllerProc() {
    // std::cout<<"###################### LQR Start ################"<<std::endl;
    //获取某一距离路径点的速度作为汽车参考速度
  double aim_l = 20.0;//目标距离
	Point_acc car_pose(ego_state_.x, ego_state_.y, ego_state_.yaw);
	TrajectoryPoint_acc car_matched_point = MatchTrajPoint(car_pose);
	auto aim_point = GetPreAimPoint(car_matched_point, aim_l);
  FDEBUG << "ACC_Aim_point_v: " << aim_point.v << std::endl;
  
  double speed_ctrl_acc = 0.0;
  double distance_ctrl_acc = 0.0;
  double output_acc = 0.0;
  //PID Speed Ctrl
  double speed_err = aim_point.v - can_info_.ego_v;
  double p_term = acc_params_.get_acc_kp() * speed_err;
  double i_term = acc_params_.get_acc_ki() * speed_err * loop_rate_;
  i_term += acc_pid_prev_i_term_;
  i_term = std::max(acc_params_.get_acc_iterm_min(), std::min(acc_params_.get_acc_iterm_max(), i_term));
  acc_pid_prev_i_term_ = i_term;

  double delta_error = speed_err - prev_speed_error_;
  double d_term = acc_params_.get_acc_kd() * delta_error;
  d_term /= loop_rate_;
  prev_speed_error_ = delta_error;

  speed_ctrl_acc = p_term + i_term + d_term;
  output_acc = speed_ctrl_acc;

  FDEBUG << "aim_point_v = " << aim_point.v << std::endl;
  FDEBUG << "ego_v = " << can_info_.ego_v << std::endl;
  FDEBUG << "speed_err = " << speed_err << std::endl;

  FDEBUG << "acc_pid_p_trem = " << p_term << std::endl;
  FDEBUG << "acc_pid_i_trem = " << i_term << std::endl;
  FDEBUG << "acc_pid_d_trem = " << d_term << std::endl;

  FDEBUG << "speed_ctrl_acc = " << speed_ctrl_acc << std::endl;
  //LQR Distance Ctrl
  if (cipv_info_.is_valid && cipv_info_.dis < 90.0) {
    FDEBUG << "**********acc control************" << std::endl;
    double D_des = can_info_.ego_v * th_ + acc_params_.get_d_offset();
    double d_error = D_des - cipv_info_.dis;
    double v_error = cipv_info_.v - can_info_.ego_v;

    FDEBUG << "D_des = " << D_des << std::endl;
    FDEBUG << "d_error = " << d_error << std::endl;
    FDEBUG << "v_error = " << v_error << std::endl;
    // MatrixInit();
    UpdateState(d_error, v_error);
    SolveLQRProblem(matrix_ad_, matrix_bd_, matrix_q_,
                    matrix_r_, acc_params_.get_tolerance(), acc_params_.get_max_num_iteration(), &matrix_k_);
    distance_ctrl_acc = -(matrix_k_ * matrix_state_)(0,0);
    if (cipv_info_.dis < acc_params_.get_d_offset() && cipv_info_.v < 0.1) {
      distance_ctrl_acc = -2.0;
    }
    FDEBUG << "distance_ctrl_acc = " << distance_ctrl_acc << std::endl;

    output_acc = std::min(speed_ctrl_acc, distance_ctrl_acc);  
  }

  FDEBUG << "output_acc = " << output_acc << std::endl;

  if (output_acc > ua_ubound_) {
    output_acc = ua_ubound_;
  } else if (output_acc < ua_lbound_) {
    output_acc = ua_lbound_;
  }
  FDEBUG << "output_acc_final= " << output_acc << std::endl;

  return output_acc;
  // double ua = 0;
  // static double ua_init = 0.2;
  // if(cipv_info_.dis > 90 || cipv_info_.v > aim_point.v || cipv_info_.is_valid == false || can_info_.ego_v > aim_point.v){
  //     FDEBUG << "**********cruise control************" << std::endl;
  //     FDEBUG << "Aim point speed is " << aim_point.v << std::endl;
  //     // u = p * dv 
  //     double pid_u = 0.5 * (aim_point.v - can_info_.ego_v);//改了v_set_   aim_point.v k = 0.2 -> 0.5
  //     double u1 = pid_u;
  //     if(can_info_.ego_v < 3.8){
  //         FDEBUG<<"ego speed is less than 3.8 ...."<<std::endl;
  //         double u1 = ua_init + 0.004;
  //         if(u1 > 2){
  //            u1 = 2;
  //         }
  //         ua_init = u1;
  //     }
  //     ua = u1 < pid_u ? u1 : pid_u;
  // } else {
  //     FDEBUG << "**********acc control************" << std::endl;
  //     ua_init = 0.0;
  //     double D_des = can_info_.ego_v * th_ + acc_params_.get_d_offset();
  //     double d_error = D_des - cipv_info_.dis;
  //     double v_error = cipv_info_.v - can_info_.ego_v;

  //     FDEBUG << "D_des = " << D_des << std::endl;
  //     FDEBUG << "d_error = " << d_error << std::endl;
  //     FDEBUG << "v_error = " << v_error << std::endl;
  //     // MatrixInit();
  //     UpdateState(d_error, v_error);
  //     SolveLQRProblem(matrix_ad_, matrix_bd_, matrix_q_,
  //                  matrix_r_, acc_params_.get_tolerance(), acc_params_.get_max_num_iteration(), &matrix_k_);
  //     ua = -(matrix_k_ * matrix_state_)(0,0);
  //     if (cipv_info_.dis < acc_params_.get_d_offset() && cipv_info_.v < 0.1) {
  //         ua = -2.0;
  //     }

  // }

  // double a_output = ua < ua_lbound_ ? ua_lbound_ : ua > ua_ubound_ ? ua_ubound_ : ua;

  // // std::cout<<" ACC Output is "<<a_output<<std::endl;

  // // std::cout<<"###################### LQR End ################"<<std::endl;

  // return a_output;
}

TrajectoryPoint_acc LqrAcc::MatchTrajPoint(const Point_acc& point) const {
	double tp_l_min = 1000.0;
	size_t tra_size = plan_trajectory_.size();
	size_t nearest_id = 0;
	for (size_t i=0; i<tra_size; ++i) {
		auto tp_l = control::math_tool::DistanceSquare(
			plan_trajectory_.at(i).x,
			plan_trajectory_.at(i).y,
			point.x, point.y);
		if (tp_l < tp_l_min) {
			tp_l_min = tp_l;
			nearest_id = i;
		}
	}
	TrajectoryPoint_acc nearest_point(plan_trajectory_.at(nearest_id).x,
		                  plan_trajectory_.at(nearest_id).y,
		                  plan_trajectory_.at(nearest_id).theta);

	return nearest_point;
}

TrajectoryPoint_acc LqrAcc::GetPreAimPoint(
	const TrajectoryPoint_acc& car_matched_point, const double pre_L) {
	size_t tra_size = plan_trajectory_.size();
	double tp_l = 0.0;

	double traj_total_s = plan_trajectory_.at(tra_size-1).s;
	double preview_s = car_matched_point.s + pre_L;

	if (traj_total_s < preview_s) {
		preview_s = traj_total_s;
	}
    // std::cout<<" conf l is "<<pre_L<<" , car s "<<car_matched_point.s<<" , preview_s "<<preview_s<<std::endl;
	auto preview_point = GetPathPointWithPathS(preview_s);
	// auto preview_point_local = OdomTransLocal(preview_point.x, preview_point.y, preview_point.theta);
	return preview_point;
}

TrajectoryPoint_acc LqrAcc::GetPathPointWithPathS(const double path_s) const {
  auto comp = [](const TrajectoryPoint_acc &tp, const double path_s) {
    return tp.s < path_s + 1e-6;
  };

  auto it_lower = std::lower_bound(plan_trajectory_.begin(),
  	                               plan_trajectory_.end(), path_s,
                                   comp);
  TrajectoryPoint_acc ptr_path_point;
  if (it_lower == plan_trajectory_.begin()) {
    ptr_path_point = plan_trajectory_.front();
  } else if (it_lower == plan_trajectory_.end()) {
    ptr_path_point = plan_trajectory_.back();
  } else {
  	ptr_path_point = control::math_tool::InterpolateUsingLinearApproximation(*(it_lower - 1),
    	*it_lower, path_s);
  }
//   std::cout<<"  ptr_path_point.x "<<ptr_path_point.x
//            <<"  ptr_path_point.y "<<ptr_path_point.y
//            <<"  ptr_path_point.theta "<<ptr_path_point.theta
//            <<"  ptr_path_point.s "<<ptr_path_point.s
//            <<"  ptr_path_point.v "<<ptr_path_point.v
//            <<std::endl;
  TrajectoryPoint_acc point_s(ptr_path_point.x, ptr_path_point.y, ptr_path_point.theta, ptr_path_point.s, ptr_path_point.v);
  return point_s;
}

void LqrAcc::SolveLQRProblem(const Matrix &A, const Matrix &B, const Matrix &Q,
                     const Matrix &R, const double tolerance,
                     const uint max_num_iteration, Matrix *ptr_K) {
    if (A.rows() != A.cols() || B.rows() != A.rows() || Q.rows() != Q.cols() ||
      Q.rows() != A.rows() || R.rows() != R.cols() || R.rows() != B.cols()) {
        return;
    }

    Matrix AT = A.transpose();
    Matrix BT = B.transpose();

    // Solves a discrete-time Algebraic Riccati equation (DARE)
    // Calculate Matrix Difference Riccati Equation, initialize P and Q
    Matrix P = Q;
    uint num_iteration = 0;
    double diff = std::numeric_limits<double>::max();
    while (num_iteration++ < max_num_iteration && diff > tolerance) {
        Matrix P_next =
            AT * P * A -
            (AT * P * B) * (R + BT * P * B).inverse() * (BT * P * A ) + Q;
        // check the difference between P and P_next
        diff = fabs((P_next - P).maxCoeff());
        P = P_next;
    }

    // if (num_iteration >= max_num_iteration) {
    //     ADEBUG << "LQR solver cannot converge to a solution, "
    //             "last consecutive result diff is: "
    //         << diff;
    // } else {
    //     ADEBUG << "LQR solver converged at iteration: " << num_iteration
    //         << ", max consecutive result diff.: " << diff;
    // }
    *ptr_K = (R + BT * P * B).inverse() * (BT * P * A );
}

