#include "vehicle_params.h"

VehicleParams::VehicleParams() {
	front_edge_to_center_ = 0.0;
	back_edge_to_center_ = 0.0;
	left_edge_to_center_ = 0.0;
	right_edge_to_center_ = 0.0;
	length_ = 0.0;
	width_ = 0.0;
	wheel_base_ = 0.0;
	min_r_ = 0.0;
	left_steer_max_ = 0.0;
	right_steer_max_ = 0.0;
	ratio_transmitting_ = 100.0;
}

VehicleParams::~VehicleParams() {}


void VehicleParams::load_left_steer_anlge_max(const double &steer) {
	left_steer_max_ = steer;
}


void VehicleParams::load_right_steer_anlge_max(const double &steer) {
	right_steer_max_ = steer;
}


void VehicleParams::load_trans_ratio(const double &ratio) {
	ratio_transmitting_ = ratio;
}


void VehicleParams::load_vehicle_size(const double &front, const double &back, 
	const double &left, const double &right, const double &l, 
	const double &w, const double &wheel_base) {
	front_edge_to_center_ = front;
	back_edge_to_center_ = back;
	left_edge_to_center_ = left;
	right_edge_to_center_ = right;
	length_ = l;
	width_ = w;
	wheel_base_ = wheel_base;
}


void VehicleParams::load_wheel_base(const double &wheel_base) {
	wheel_base_ = wheel_base;
}


double VehicleParams::get_wheel_base() {
	return wheel_base_;
}


double VehicleParams::get_trans_ratio() {
	return ratio_transmitting_;
}

double VehicleParams::get_left_steer_anlge_max() {
	return left_steer_max_;
}


double VehicleParams::get_right_steer_anlge_max() {
	return right_steer_max_;
}

double VehicleParams::get_left_wheel_anlge_max() {
	return (left_steer_max_ / ratio_transmitting_);
}


double VehicleParams::get_right_wheel_anlge_max() {
	return (right_steer_max_ / ratio_transmitting_);
}
