#include "ego_state.h"

State::State():begin(false)
{
	// this->max_delta_angle = 500.0f / this->ratio_transmitting;//(left -520,right:500)
	// this->min_delta_angle = -520.0f / this->ratio_transmitting;
	// this->max_delta_arc = this->max_delta_angle / 180.0f * M_PI;
	// this->min_delta_arc = this->min_delta_angle / 180.0f * M_PI;
	this->x = 0.0;
	this->y = 0.0;
	this->yaw = 0.0;
	this->speed = 0.0;
}

State::~State(){ }

void State::initial(double steer_max_l, double steer_max_r, double steer_trans_ratio, double wheel_base) {
	if (!this->begin){
        this->begin = true;
		this->x = 0.0;
	    this->y = 0.0;
	    this->yaw = 0.0; //deeg
	    this->speed = 0.0;
	    this->yaw_rate = 0.0; //deeg/s
            this->steer_angle = 0.0;
	    this->ratio_transmitting = steer_trans_ratio;
		this->max_delta_angle = steer_max_r / this->ratio_transmitting;//(left -520,right:500)
		this->min_delta_angle = steer_max_l / this->ratio_transmitting;
		this->max_delta_arc = this->max_delta_angle / 180.0f * M_PI;
		this->min_delta_arc = this->min_delta_angle / 180.0f * M_PI;
		this->distance_between_wheel = wheel_base;
	}
	
}

void State::normalize_angle_deeg(float &yaw) {
	while (yaw > 180.0) {
		yaw -= 360.0;
	}

	while (yaw < -180.0) {
		yaw += 360.0;
	}

}

void State::update_pose(const float &x, const float &y, const double &yaw, const float &speed, const double &dt)
{
	this->x += speed * std::cos(yaw) * dt;
	this->y += speed * std::sin(yaw) * dt;
	this->yaw = yaw;
	this->speed = speed;
}

// void State::Load_vehicle_params(double steer_max_l, double steer_max_r, double steer_trans_ratio, double wheel_base) {
// 	this->ratio_transmitting = steer_trans_ratio;
// 	this->max_delta_angle = steer_max_r / this->ratio_transmitting;//(left -520,right:500)
// 	this->min_delta_angle = steer_max_l / this->ratio_transmitting;
// 	this->max_delta_arc = this->max_delta_angle / 180.0f * M_PI;
// 	this->min_delta_arc = this->min_delta_angle / 180.0f * M_PI;
// 	this->distance_between_wheel = wheel_base;
// }


void State::load_pose_value(const float &x, const float &y,const double &yaw, const float &speed){
	this->x = x;
	this->y = y;
	this->yaw = yaw;
	this->speed = speed;
}

void State::normalize_delta(float &delta) {
	delta = - delta / this->distance_between_wheel;
	if (delta > this->max_delta_angle)
	{
		delta = this->max_delta_angle;
	}
	else if (delta < this->min_delta_angle)
	{
		delta = this->min_delta_angle;
	}
	else
	{
		delta = delta;
	}
}

void State::update(float &acceleration, float &delta, float &dt) {
	this->normalize_delta(delta);
	this->x += this->speed * std::cos(this->yaw) * dt;
	this->y += this->speed * std::sin(this->yaw) * dt;
	this->yaw += this->speed / this->distance_between_wheel * std::tan(delta) * dt;
	this->normalize_angle_deeg(this->yaw);
	this->speed += acceleration * dt;
}
