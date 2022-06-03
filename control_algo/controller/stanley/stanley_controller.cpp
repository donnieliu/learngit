#include "stanley_controller.h"

StanleyController::StanleyController() : k(0.1f),
kp(0.20f),
distance_threshold(0.20f),
speed_threshold(2.20f+1e-6) {

}

StanleyController::~StanleyController() { }

void StanleyController::pid_control(float &target_speed, float &current_speed, double &acceleration) {
	acceleration = this->kp * (target_speed - current_speed);
}

void StanleyController::calculate_target_index(icvos::ADCTrajectory &trajectory_, int &target_index, float &error_front_axle, bool &path_judge){
	
	if (trajectory_.adc_trajectory_path().empty())
	{
		std::cout << "Trajectory is empty!......." << std::endl;
		return;
	}

    float fx = this->ego_state.x + this->ego_state.distance_between_wheel * std::cos(this->ego_state.yaw);
	float fy = this->ego_state.y + this->ego_state.distance_between_wheel * std::sin(this->ego_state.yaw);
	float min_value(1000.0f);

	int trajectory_points = static_cast<int>(trajectory_.adc_trajectory_path().size());
	for (int i=0; i<trajectory_points; ++i)
	{
		float distance = std::sqrt((fx - trajectory_.adc_trajectory_path(i).x()) * (fx - trajectory_.adc_trajectory_path(i).x()) + (fy - trajectory_.adc_trajectory_path(i).y()) * (fy - trajectory_.adc_trajectory_path(i).y()));
		if (distance < min_value)
		{
			min_value = distance;
			target_index = i;
		}
	}

	Eigen::Vector2f front_axle_vec;
	front_axle_vec << std::cos(this->ego_state.yaw - M_PI/2), std::sin(this->ego_state.yaw - M_PI/2);
    Eigen::Vector2f ego_pos;
	ego_pos << trajectory_.adc_trajectory_path(target_index).x() - fx, trajectory_.adc_trajectory_path(target_index).y() - fy;
	error_front_axle = front_axle_vec.dot(ego_pos);
	if (std::fabs(error_front_axle) <= this->distance_threshold)
	{
      error_front_axle = 0.0f;
	}
	else
	{
		if (error_front_axle > this->distance_threshold) error_front_axle -= this->distance_threshold;
		else if (error_front_axle < -this->distance_threshold) error_front_axle += this->distance_threshold;
	} 
	path_judge = true;
	std::cout << "error_front_axle=" << error_front_axle << std::endl;
}

void StanleyController::stanley_control(icvos::ADCTrajectory &trajectory_, double &delta, double &acceleration){
    bool path(false);
	float error_front_axle(0.0f);
	int target_index(-1);
	this->calculate_target_index(trajectory_, target_index, error_front_axle, path);
	if (path)
	{

		double theta_e = (trajectory_.adc_trajectory_path(target_index).theta() / 180 * M_PI) - this->ego_state.yaw;
        this->normalize_angle(theta_e);
		float current_speed(0.0f);
		if (this->ego_state.speed <= this->speed_threshold){
			current_speed = this->speed_threshold;
		}
		else{
			current_speed = this->ego_state.speed;
		}
	    double theta_d = std::atan2(this->k * error_front_axle, current_speed);
		delta = theta_e + theta_d;
		this->normalize_angle(delta);

        std::cout << "theta_d=" << theta_d << ";theta_e=" << theta_e << std::endl; 
		std::cout << "---------------------------" << std::endl;
		
        float target_speed = (float)trajectory_.adc_trajectory_path(target_index).speed();
	    this->pid_control(target_speed, this->ego_state.speed, acceleration);

	}
	else{
		std::cout << " Can not find nearest trajectory point!Error!.........." << std::endl;
	}
}



void StanleyController::normalize_angle(double &yaw) {
	while (yaw > M_PI) {
		yaw -= 2.0 * M_PI;
	}

	while (yaw < -M_PI) {
		yaw += 2.0 * M_PI;
	}

}