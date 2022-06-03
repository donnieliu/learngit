#pragma once
#include <cmath>
#include <math.h>

class VehicleParams
{
public:
	VehicleParams();
	~VehicleParams();
	void load_left_steer_anlge_max(const double &steer);//(left -520,right:500)
	void load_right_steer_anlge_max(const double &steer);
	void load_trans_ratio(const double &ratio);
	void load_vehicle_size(const double &front, const double &back, 
		const double &left, const double &right, const double &l, 
		const double &w, const double &wheel_base);
	void load_wheel_base(const double &wheel_base);

	double get_wheel_base();
	double get_trans_ratio();
	double get_left_steer_anlge_max();
	double get_right_steer_anlge_max();
	double get_left_wheel_anlge_max();
	double get_right_wheel_anlge_max();
private:
	double front_edge_to_center_;
	double back_edge_to_center_;
	double left_edge_to_center_;
	double right_edge_to_center_;
	double length_;
	double width_;
	double wheel_base_;
	double min_r_;
	double left_steer_max_;
	double right_steer_max_;
	double ratio_transmitting_;	
};