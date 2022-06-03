#pragma once
#include <cmath>
#include <math.h>
struct lqr_q
{
	float q1;
	float q2;
	float q3;
	float q4;
};
struct lqr_r
{
	float r1;
};

class AccParams
{
public:
	AccParams();
	~AccParams();
	void load_ts(const double &ts);
	void load_d_offset(const double &doffset);
	void load_state_size(const double &state_size);
    void load_tolerance(const double &tolerance);
    void load_max_num_iteration(const double &max_num_iteration);
    void load_q(const float& q1, const float& q2, const float& q3, const float& q4);
    void load_r(const float& r1);

    void load_acc_kp(const double& acc_kp);
    void load_acc_ki(const double& acc_ki);
    void load_acc_kd(const double& acc_kd);
    void load_acc_itrem_max(const double& acc_iterm_max);
    void load_acc_itrem_min(const double& acc_iterm_min);

	double get_ts() {return ts_;};
	double get_d_offset() {return d_offset_;};
	double get_state_size() {return state_size_;};
	double get_tolerance() {return tolerance_;};
    double get_max_num_iteration() {return max_num_iteration_;};

    double get_acc_kp() {return acc_kp_;};
    double get_acc_ki() {return acc_ki_;};
    double get_acc_kd() {return acc_kd_;};
    double get_acc_iterm_max() {return acc_iterm_max_;};
    double get_acc_iterm_min() {return acc_iterm_min_;};

    float get_q1() {return q_.q1;};
    float get_q2() {return q_.q2;};
    float get_q3() {return q_.q3;};
    float get_q4() {return q_.q4;};
    float get_r1() {return r_.r1;};

private:
	double ts_;
	double d_offset_;
    double state_size_;
    double tolerance_;
    double max_num_iteration_;

    double acc_kp_;
    double acc_ki_;
    double acc_kd_;
    double acc_iterm_max_;
    double acc_iterm_min_;
    lqr_q q_;
    lqr_r r_;

};
