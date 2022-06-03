#pragma once
#include <iostream>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <opencv2/core/core.hpp>

#include "log.h"

namespace  eps_ctrl{
typedef struct
{
    float step_max;
    float outmax;
    float outmin;
    float torque_rate_max;
}EPS_LIMIT_TYPE;


class PID
{
public:
	PID() = default;
	~PID()= default;
	void Init(std::string *config_path);
	void Reset();
	void CancelReset();
	int is_reset() {return reset_flag_;};
	int PidCtrl(const double& target, const double& actual, const double& speed);
	double get_result() {return result_;};
	void epsTorqueLimitFunc(float *eps_torque);
	
private:
	int LoadParam(std::string *config_path);
	double FeedForward(const double& target_angle, const double& actual_speed);


private:
	double kp_;
	double ki_;
	double kd_;
	double ki_limit_;
	double loop_rate_;
	int reset_flag_;	

	double prev_angle_error_;
	double prev_i_term_;

	double K_beta_;
	double K_alpha_;
	double v_max_; //km/h
	double init_torque_;
	double result_;

	EPS_LIMIT_TYPE eps_limit_;

};

}