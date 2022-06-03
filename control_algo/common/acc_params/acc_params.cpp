#include "acc_params.h"
#include <iostream>

AccParams::AccParams() {
	d_offset_ = 0.0;
	state_size_ = 0.0;
	tolerance_ = 0.0;
	max_num_iteration_ = 0.0;
	acc_iterm_max_ = 0.0;
	acc_iterm_min_ = 0.0;
}

AccParams::~AccParams() {}

void AccParams::load_ts(const double &ts) {
	ts_ = ts;
}

void AccParams::load_d_offset(const double &doffset) {
	d_offset_ = doffset;
}

void AccParams::load_state_size(const double &state_size) {
	state_size_ = state_size;
}

void AccParams::load_tolerance(const double &tolerance) {
	tolerance_ = tolerance;
}

void AccParams::load_max_num_iteration(const double &max_num_iteration) {
	max_num_iteration_ = max_num_iteration;
}

void AccParams::load_acc_kp(const double& acc_kp) {
	acc_kp_ = acc_kp;
}

void AccParams::load_acc_ki(const double& acc_ki) {
	acc_ki_ = acc_ki;
}

void AccParams::load_acc_kd(const double& acc_kd) {
	acc_kd_ = acc_kd;
}

void AccParams::load_acc_itrem_max(const double& acc_iterm_max) {
	acc_iterm_max_ = acc_iterm_max;
}

void AccParams::load_acc_itrem_min(const double& acc_iterm_min) {
	acc_iterm_min_ = acc_iterm_min;
}

void AccParams::load_q(const float& q1, const float& q2, const float& q3, const float& q4) {
	q_.q1 = q1;
	q_.q2 = q2;
	q_.q3 = q3;
	q_.q4 = q4;
}
void AccParams::load_r(const float& r1) {
	r_.r1 = r1;
}
