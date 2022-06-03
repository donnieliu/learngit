#pragma once
#define _USE_MATH_DEFINES
#include <cmath>
#include <math.h>

class State {
public:
	State();
	~State();
	float distance_between_wheel;
	float ratio_transmitting;
	float x;
	float y;
	float yaw;//deeg
	float yaw_rate; //deeg/s
	float steer_angle;//deeg
	float speed;
	float min_delta_angle;
	float max_delta_angle;
	float min_delta_arc;
	float max_delta_arc;
	bool begin;
	void initial(double steer_max_l, double steer_max_r, double steer_trans_ratio, double wheel_base);
	void update_pose(const float &x, const float &y, const double &yaw, const float &speed, const double &dt);
	void load_pose_value(const float &x, const float &y, const double &yaw, const float &speed);
	// void Load_vehicle_params(double steer_max_l, double steer_max_r, double steer_trans_ratio, double wheel_base);
	void update(float &acceleration, float &delta, float &dt);
	void normalize_angle_deeg(float &yaw);
	void normalize_delta(float &delta);
};
