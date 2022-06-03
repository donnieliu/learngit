#pragma once
#include <iostream>
#include <Eigen/Core>


#include "ego_state/ego_state.h"

class StanleyController {
public:
	StanleyController();
	~StanleyController();
	void pid_control(float &target_speed, float &current_speed, double &acceleration);
	void calculate_target_index(pbTrajectoryMsg &trajectory_, int &target_index,  float &error_front_axle, bool &path_judge);
    void stanley_control(pbTrajectoryMsg &trajectory_, double &delta, double &acceleration);
	void normalize_angle(double &yaw);
	//void stanley_control_main(icvos::ADCTrajectory &trajectory_, )
	State ego_state;
private:
	float k; // control gain
	float kp; // speed proportional gain
	float speed_threshold;
	float distance_threshold;
};
