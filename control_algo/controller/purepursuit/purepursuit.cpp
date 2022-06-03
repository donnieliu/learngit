#include "purepursuit.h"
PurePursuit::PurePursuit() :
	kv_(0.2f),
	kl_(1.0f),
	dl_(6.0f),
	pre_l_min_(6.0),
	pre_l_max_(30.0){
}


PurePursuit::~PurePursuit() {}


int PurePursuit::Init(std::string *file_path) {
	output_a_ = 0;
	output_steer_angle_ = 0;
	LoadConf(file_path);
	ego_state_.initial(ego_params_.get_left_steer_anlge_max(), 
		               ego_params_.get_right_steer_anlge_max(), 
		               ego_params_.get_trans_ratio(), 
		               ego_params_.get_wheel_base());
	park_length_ = 0.0;
	track_error_record_.first = 0.0;
	track_error_record_.second = 0.0;
                          
	return 0;
}

int PurePursuit::LoadConf(std::string *file_path) {
	YAML::Node config = YAML::LoadFile(*file_path);

	steer_step_max_ = config["EpsParams"]["steer_step_max"].as<double>();

	ego_params_.load_left_steer_anlge_max(config["VehicleParams"]["l_steer_max"].as<double>());
	ego_params_.load_right_steer_anlge_max(config["VehicleParams"]["r_steer_max"].as<double>());
	ego_params_.load_trans_ratio(config["VehicleParams"]["steer_ratio"].as<double>());
	ego_params_.load_wheel_base(config["VehicleParams"]["wheel_base"].as<double>());
	FDEBUG << "purepursuit config " << ego_params_.get_trans_ratio() << " , "
                                    << ego_params_.get_wheel_base() << " , "
                                    << ego_params_.get_right_steer_anlge_max() << " , "
                                    << ego_params_.get_left_steer_anlge_max() << " , "
                                    << steer_step_max_
                                    << std::endl;
    return 0;
}

int PurePursuit::LoadData(const TrajectoryPlaningMsg* trajectory,
	const LocationMsg *ego_pose, const VehicleReportMsg* ego_can) {
	plan_trajectory_.clear();
	int tra_size = trajectory->trajectory_points_size;
	for (size_t i=0; i<tra_size; ++i) {
		auto tp_p = trajectory->trajectory_points[i];
		TrajectoryPoint tp_tra_p(tp_p.way_point.x, tp_p.way_point.y,
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

	ego_state_.yaw_rate = ego_can->chassis_report.ego_motion.yaw_rate; //deeg/s
	double ego_speed = ego_can->chassis_report.velocity;//for simulation
	ego_state_.steer_angle = ego_can->chassis_report.steer_angle;
	car_speed_ = ego_speed;
	ego_state_.load_pose_value(ego_x, ego_y, ego_yaw, ego_speed);

  	FDEBUG << "c0 -- c3 "<<trajectory->coe.c0<<" , "<<trajectory->coe.c1<<" , "<<trajectory->coe.c2<<" , "<<trajectory->coe.c3<<std::endl;

//   std::cout<<">>>traj info "<<std::endl;
//   std::cout<<"             traj size  : "<<plan_trajectory_.size()<<std::endl;
  // int ii = plan_trajectory_.size();
  // if (ii > 10){
  //     ii = 10;
  // }
//   for(int i=0; i<ii; ++i){
//       std::cout<<"         No."<<i
//                <<" , x : "<<plan_trajectory_.at(i).x
//                <<" , y : "<<plan_trajectory_.at(i).y
//                <<" , yaw : "<<plan_trajectory_.at(i).theta
//                <<" , s : "<<plan_trajectory_.at(i).s
//                <<" , v : "<<plan_trajectory_.at(i).v<<std::endl;
//   }

	return 0;
}


int PurePursuit::ControllerProc() {
	if (CheckIn()) {
		output_a_ = 0.0;
		output_steer_angle_ = 0.0;
		return 0;
	}

	double aim_l = kl_ * ego_state_.speed + dl_;
	if (aim_l < pre_l_min_) {
		aim_l = pre_l_min_;
	} else if (aim_l > pre_l_max_) {
		aim_l = pre_l_max_;
	}
	Point car_pose(ego_state_.x, ego_state_.y, ego_state_.yaw);
	TrajectoryPoint car_matched_point = MatchTrajPoint(car_pose);
	auto track_error = GetTrackError(car_pose, car_matched_point);

	auto aim_point = GetPreAimPoint(car_matched_point, aim_l);
	double time_delay = 0.0;
	auto ego_car_future_position = EstimateFuturePosition(time_delay);
	double deta_l_predict = control::math_tool::DistanceSquare(
		ego_car_future_position.x, ego_car_future_position.y, ego_state_.x, ego_state_.y);

	if(deta_l_predict > 15.0 ||
	   fabs(ego_car_future_position.theta - ego_state_.yaw) > 15.0) {
		FDEBUG  << "Error : there is big gap between ego_position and future position!"
				    << " x gap "<<ego_car_future_position.x - ego_state_.x
				    << " y gap "<<ego_car_future_position.y - ego_state_.y
				    << " theta gap "<<ego_car_future_position.theta - ego_state_.yaw << std::endl;
		ego_car_future_position.x = ego_state_.x;
		ego_car_future_position.y = ego_state_.y;
		ego_car_future_position.theta = ego_state_.yaw;
		track_error.first = track_error_record_.first;
		track_error.second = track_error_record_.second;
	}

	LonControllerProc(aim_point, track_error.first, track_error.second);

	LatControllerProc(ego_car_future_position, aim_point, track_error.first, track_error.second);

	track_error_record_.first = track_error.first;
	track_error_record_.second = track_error.second;


#ifdef SIM_DEBUG
 //  	std::cout<<" base length "<<ego_params_.get_wheel_base()
	//          <<" ratio is "<<ego_state_.ratio_transmitting<<std::endl;

	// std::cout << "car_matched_point " << car_matched_point.x << " , "
	//                                    << car_matched_point.y << " , "
	//                                    << car_matched_point.theta << std::endl;
	// std::cout << "track_error : " << track_error.first << " , " << track_error.second << std::endl;
	// std::cout << "aim point is " << aim_point.x << " , " << aim_point.y << " , " << aim_point.theta << std::endl;
	// std::cout << "ego_car_future_position : " << ego_car_future_position.x << " , "
	//                                            << ego_car_future_position.y << " , "
	//                                            << ego_car_future_position.theta << std::endl;
#endif

	return 0;
}


// Point PurePursuit::EstimateFuturePosition(const double t) const { //yaw_rate
//   Point vec_distance;
//   double d_x = 0.0;
//   double d_y = 0.0;
//   double deta = 0.0;
//   double v = ego_state_.speed;
//   // Predict distance travel vector
//   if (std::fabs(ego_state_.yaw_rate) < 0.0001) {
//     d_x = v * t;
//     d_y = 0.0;
//     deta = 0.0;
//   } else {
//     double rr = v / (ego_state_.yaw_rate * M_PI / 180.0);
//     double d_deta = ego_state_.yaw_rate * M_PI / 180.0 * t;
//     d_y = -rr * (1.0 - std::cos(d_deta));
//     d_x = std::sin(d_deta) * rr;
//     deta = d_deta * 180.0 / M_PI;
//   }
//   Point ego_car_odom, target_local, target_odom;
//   ego_car_odom.x  = ego_state_.x;
//   ego_car_odom.y  = ego_state_.y;
//   ego_car_odom.theta  = ego_state_.yaw;

//   target_local.x = d_x;
//   target_local.y = d_y;
//   target_local.theta = deta;

//   LocalTransOdom(ego_car_odom, target_local, target_odom);
//   std::cout<<"future position local: "<<target_local.x<<" , "
//                                       <<target_local.y<<" , "
//                                       <<target_local.theta
//                                       <<std::endl;
//   std::cout<<"future position odom : "<<target_odom.x<<" , "
//                                       <<target_odom.y<<" , "
//                                       <<target_odom.theta
//                                       <<std::endl;
//   return target_odom;
// }

Point PurePursuit::EstimateFuturePosition(const double t) const {//steer_angle
  double v = ego_state_.speed;
  double steer_rad = ego_state_.steer_angle / ego_state_.ratio_transmitting * DEG_TO_RAD;
  double fi = ego_state_.yaw * DEG_TO_RAD;
  double d_x = v * std::cos(fi) * t;
  double d_y = v * std::sin(fi) * t;
  double d_fi = - v * std::tan(steer_rad) / ego_state_.distance_between_wheel;
  Point ego_car_odom, target_odom, target_local;
  ego_car_odom.x  = ego_state_.x;
  ego_car_odom.y  = ego_state_.y;
  ego_car_odom.theta  = ego_state_.yaw;

  target_odom.x = d_x + ego_car_odom.x;
  target_odom.y = d_y + ego_car_odom.y;
  target_odom.theta = d_fi + ego_car_odom.theta;
// #ifdef SIM_DEBUG
//   OdomTransLocal(ego_car_odom, target_odom, target_local);

//   std::cout << "future position odom : "<<target_odom.x<<" , "
//                                       <<target_odom.y<<" , "
//                                       <<target_odom.theta << std::endl;
//   std::cout << "future position local: "<<target_local.x<<" , "
//                                       <<target_local.y<<" , "
//                                       <<target_local.theta << std::endl;
// #endif
    return target_odom;
}


double PurePursuit::GetTargetAcceleration() {
	return output_a_;
}


double PurePursuit::GetTargetSteerAngle() {
	return output_steer_angle_;
}


int PurePursuit::CheckIn() {
	if (plan_trajectory_.empty() ||
		plan_trajectory_.size() < 1) {
		FERROR<<"Input trajectory is empty!" << std::endl;
		return -1;
	}
	return 0;
}

double PurePursuit::GetParkLength() {
	double park_length = 0.0;
	size_t tra_size = plan_trajectory_.size();
	double tp_l = 0.0;
	for (size_t i=0; i<tra_size; ++i) {
		auto tp_p = plan_trajectory_.at(i);
		if (tp_p.v < kMathEpsilon && tp_p.s > 1.0) {
			park_length = tp_p.s;
			return park_length;
		}
	}
	return park_length;
}


TrajectoryPoint PurePursuit::GetPreAimPoint(
	const TrajectoryPoint& car_matched_point, const double pre_L) {
	size_t tra_size = plan_trajectory_.size();
	double tp_l = 0.0;

	double traj_total_s = plan_trajectory_.at(tra_size-1).s;
	double preview_s = car_matched_point.s + pre_L;

	if (traj_total_s < preview_s) {
		preview_s = traj_total_s;
	}

	auto preview_point = GetPathPointWithPathS(preview_s);
	return preview_point;
}

int PurePursuit::LonControllerProc(const TrajectoryPoint& aim_point,
	double lat_error, double angle_error) {

	double park_length = GetParkLength();
	double aim_speed = aim_point.v;
	if(!(fabs(lat_error) < 1.0 && fabs(angle_error) * 180 / M_PI < 30.0)){
	}

	double deta_v = aim_speed - ego_state_.speed;
	if (deta_v > 1.0) {
		output_a_ = 2 * kv_ * deta_v;

	} else if (deta_v < -1.0) {
		output_a_ = 2* kv_ * deta_v;
	} else {
		output_a_ = kv_ * deta_v;
	}
	return 0;
}


int PurePursuit::LatControllerProc(const Point& ego_car_position, const TrajectoryPoint& aim_point,
	double lat_error, double angle_error) {
	static double output_deta_last = 0.0; //rad
    static double target_angle_pre = 0.0; // deeg

	Point ego_car_odom(ego_car_position.x, ego_car_position.y, ego_car_position.theta);
	Point target_odom(aim_point.x, aim_point.y, aim_point.theta);
	Point target_local;

	OdomTransLocal(ego_car_odom, target_odom, target_local);
	double alpha_rad = std::atan2(target_local.y, target_local.x);
	double x_0 = 0.0;
	double y_0 = 0.0;
	double arc_r = 1000;
	double num = x_0 * x_0 + y_0 * y_0 - target_local.x * target_local.x - target_local.y * target_local.y;
	double den = 2 * (y_0 - target_local.y);

	double wheel_angle = 0.0; //rad
	if (fabs(den) < 0.01) {
		wheel_angle = 0.0; 
	} else {
		arc_r = num / den;
		if (fabs(arc_r) > 5) {
			wheel_angle = std::atan2(ego_params_.get_wheel_base(), fabs(arc_r));
			if (arc_r < 0.0) {
				wheel_angle = - wheel_angle;
			}
		} else {
			wheel_angle = output_deta_last;
		}
	}

	output_deta_last = wheel_angle;

    double steer_angle = wheel_angle / M_PI * 180.0 * ego_params_.get_trans_ratio();//逆时针为正 deeg
    if (steer_angle > 0.0 && steer_angle > ego_params_.get_right_steer_anlge_max())
    {
      steer_angle = ego_params_.get_right_steer_anlge_max();
    }
    else if (steer_angle < 0.0 && steer_angle < -ego_params_.get_left_steer_anlge_max())
    {
      steer_angle = -ego_params_.get_left_steer_anlge_max();
    }

    if (steer_angle - target_angle_pre > steer_step_max_) {
      steer_angle = target_angle_pre + steer_step_max_;
    } else if (steer_angle - target_angle_pre < -steer_step_max_) {
      steer_angle = target_angle_pre - steer_step_max_;
    }
    target_angle_pre = steer_angle;

    output_steer_angle_ = steer_angle;


// #ifdef SIM_DEBUG
// 	std::cout<<"car pose : "<<ego_state_.x<<" , "<<ego_state_.y<<" , "<<ego_state_.yaw<<std::endl;
// 	std::cout<<"aim point odom : "<<aim_point.x<<" , "<<aim_point.y<<" , "<<aim_point.theta<<" , "<<aim_point.s<<std::endl;
// 	std::cout<<"aim point local : "<<target_local.x<<" , "<<target_local.y<<" , "<<target_local.theta<<std::endl;
// 	std::cout<<"alpha_rad : "<<alpha_rad*180/M_PI<<" | "<<target_local.theta<<std::endl;
// 	std::cout<<"arc_r1 : "<<arc_r<<std::endl;
// 	std::cout<<"output_steer_angle_ : "<<output_steer_angle_<<std::endl;
// #endif
	return 0;
}


int PurePursuit::OdomTransLocal(
	const Point& ego_car_odom, const Point& target_odom, Point& target_local) const {
	double deta_x = target_odom.x - ego_car_odom.x;
	double deta_y = target_odom.y - ego_car_odom.y;

	double ego_yaw_rad = ego_car_odom.theta * DEG_TO_RAD;
	double ego_x =   deta_x * std::cos(ego_yaw_rad) + deta_y * std::sin(ego_yaw_rad);
	double ego_y = - deta_x * std::sin(ego_yaw_rad) + deta_y * std::cos(ego_yaw_rad);

	double ego_theta = target_odom.theta - ego_car_odom.theta;
	target_local.x = ego_x;
	target_local.y = ego_y;
	target_local.theta = control::math_tool::NormalizeAngle_deeg(ego_theta);
	return 0;
}


int PurePursuit::LocalTransOdom(const Point &ego_car_odom,
	const Point &target_local, Point& target_odom) const {
	double rad = ego_car_odom.theta * DEG_TO_RAD;
	double target_x = target_local.x * cos(rad) - target_local.y * sin(rad);
	double target_y = target_local.x * sin(rad) + target_local.y * cos(rad);
	target_x += ego_car_odom.x;
	target_y += ego_car_odom.y;
	double target_angle = ego_car_odom.theta + target_local.theta;
	target_odom.x = target_x;
	target_odom.y = target_y;
	target_odom.theta = target_angle;
	target_odom.theta = control::math_tool::NormalizeAngle_deeg(target_odom.theta);

	return 0;
}


TrajectoryPoint PurePursuit::MatchTrajPoint(const Point& point) const {
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
	// TrajectoryPoint nearest_point(plan_trajectory_.at(nearest_id).x,
	// 	                  plan_trajectory_.at(nearest_id).y,
	// 	                  plan_trajectory_.at(nearest_id).theta);
	auto point_sd = ComputePositionProjection(ego_state_.x, ego_state_.y, plan_trajectory_.at(nearest_id));

	return (GetPathPointWithPathS(point_sd.first));
}

std::pair<double, double> PurePursuit::GetTrackError(const Point reference_point_odom,
	const TrajectoryPoint& traj_point) const {
	double lat_err = 0.0;
	double angle_err = 0.0;

	Point target_odom(traj_point.x, traj_point.y, traj_point.theta);
	Point target_local;
	OdomTransLocal(reference_point_odom, target_odom, target_local);

	lat_err = target_local.y;//std::sqrt(target_local.x * target_local.x + target_local.y * target_local.y);

	angle_err = target_local.theta; //deeg
 	return std::make_pair(lat_err, angle_err);
}

std::pair<double, double> PurePursuit::ComputePositionProjection(
    const double x, const double y, const TrajectoryPoint& p) const {

    double v_x = x - p.x;
    double v_y = y - p.y;

    double n_x = std::cos(p.theta);
    double n_y = std::sin(p.theta);

    Point v(x - p.x, y - p.y);
    Point n(std::cos(p.theta), std::sin(p.theta));

    std::pair<double, double> frenet_sd;
    frenet_sd.first = control::math_tool::InnerProd(v.x, v.y, n.x,
        n.y) + p.s;
    frenet_sd.second = control::math_tool::CrossProd(v.x, v.y, n.x, n.y);

    return frenet_sd;
}


TrajectoryPoint PurePursuit::GetPathPointWithPathS(const double path_s) const {
  auto comp = [](const TrajectoryPoint& tp, const double path_s) {
    return tp.s < path_s + 1e-6;
  };

  auto it_lower = std::lower_bound(plan_trajectory_.begin(),
  	                               plan_trajectory_.end(), path_s,
                                   comp);

  TrajectoryPoint ptr_path_point;
  if (it_lower == plan_trajectory_.begin()) {
    ptr_path_point = plan_trajectory_.front();
  } else if (it_lower == plan_trajectory_.end()) {
    ptr_path_point = plan_trajectory_.back();
  } else {
  	ptr_path_point = control::math_tool::InterpolateUsingLinearApproximation(*(it_lower - 1),
    	*it_lower, path_s);
  }
  TrajectoryPoint point_s(ptr_path_point.x, ptr_path_point.y, ptr_path_point.theta, ptr_path_point.s, ptr_path_point.v);
  return point_s;
}
