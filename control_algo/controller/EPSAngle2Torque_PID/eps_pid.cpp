#include "eps_pid.h"

#include "mathtool/math_utils.h"

#include <yaml-cpp/yaml.h>

namespace  eps_ctrl{

void PID::Init(std::string *config_path) {
	kp_ = 1.0;
	ki_ = 1.0;
	kd_ = 0.0;
	ki_limit_ = 10;
	loop_rate_ = 0.01; //s
	reset_flag_ = 0;
    K_beta_ = 3.3 * 1e-5;
    K_alpha_ = 1.4;
    v_max_ = 180; // km/h
    init_torque_ = 0.0;
	LoadParam(config_path);
	Reset();
}

void PID::Reset() {
	FDEBUG << "pid reset !" <<std::endl;
	prev_i_term_ = 0;
	reset_flag_ = 1;
}

void PID::CancelReset() {
	FDEBUG << "cancle pid reset !" <<std::endl;
	reset_flag_ = 0;
}


int PID::PidCtrl(const double& target, const double& actual, const double& speed) {// deeg  m/s
    double angle_error = target - actual;
    double p_term = kp_ * angle_error;
    double i_term = ki_ * angle_error * loop_rate_;
    i_term += prev_i_term_;
    i_term = std::max(-ki_limit_, std::min(ki_limit_, i_term));

    prev_i_term_ = i_term;

    double delta_error = angle_error - prev_angle_error_;
    double d_term = kd_ * delta_error;
    d_term /= loop_rate_;

    prev_angle_error_ = angle_error;

    double ff_result = FeedForward(target, speed);
    result_ = p_term + i_term + d_term + ff_result;

    FERROR<<" PID DEBUG "<<std::endl;
    FERROR<<" PID CTRL target_angle : "<<target<<std::endl;
    FERROR<<" PID CTRL actual_angle : "<<actual<<std::endl;
    FERROR<<" PID CTRL error : "<<angle_error<<std::endl;
    FERROR<<" PID CTRL speed : "<<speed<<std::endl;
    FDEBUG<<" PID CTRL p_term "<<p_term<<std::endl;
    FDEBUG<<" PID CTRL i_term "<<i_term<<std::endl;
    FDEBUG<<" PID CTRL d_term "<<d_term<<std::endl;
    FDEBUG<<" PID CTRL ff_result "<<ff_result<<std::endl;
    FERROR<<" PID CTRL (pid result) : "<<result_<<std::endl;

    return 0;
}

int PID::LoadParam(std::string *config_path) {
    YAML::Node config = YAML::LoadFile(*config_path);
    kp_          = config["EpsPIDCtrl"]["kp"].as<double>();
    ki_          = config["EpsPIDCtrl"]["ki"].as<double>();
    kd_          = config["EpsPIDCtrl"]["kd"].as<double>();
    ki_limit_    = config["EpsPIDCtrl"]["ki_max"].as<double>();
    K_beta_      = config["EpsPIDCtrl"]["K_beta"].as<double>();
    K_alpha_     = config["EpsPIDCtrl"]["K_alpha"].as<double>();
    v_max_       = config["EpsPIDCtrl"]["v_max"].as<double>();
    init_torque_ = config["EpsPIDCtrl"]["init_torque"].as<double>();
    loop_rate_   = config["NodeSet"]["ctrl_loop"].as<double>();
    
    FDEBUG << "eps_pid load param !" << std::endl;
    FDEBUG <<" kp "<<kp_<<std::endl;
    FDEBUG <<" ki "<<ki_<<std::endl;
    FDEBUG <<" kd "<<kd_<<std::endl;
    FDEBUG <<" ki_limit "<<ki_limit_<<std::endl;
    FDEBUG <<" K_beta "<<K_beta_<<std::endl;
    FDEBUG <<" K_alpha "<<K_alpha_<<std::endl;
    FDEBUG <<" loop_rate "<<loop_rate_<<std::endl;

    return 0;
}

double PID::FeedForward(const double& target_angle, const double& actual_speed) {
    double feed_forward = 0.0;
    double actual_speed_km = actual_speed * 3.6;
    double y_v  = - K_beta_ * actual_speed_km * actual_speed_km * (actual_speed_km / 3.0 - v_max_ / 2.0) + init_torque_;
    double y_sw = K_alpha_ * std::sqrt(fabs(target_angle));
    if (target_angle < -2.0 || target_angle > 2.0) {
      y_v = fabs(y_v) * control::math_tool::sign(target_angle);
    } else {
      y_v = fabs(y_v) * target_angle / 2.0;
    }
    y_sw = fabs(y_sw) * control::math_tool::sign(target_angle);
    feed_forward = y_v + y_sw;
    FERROR<<"y_v : "<<y_v<<std::endl;
    FERROR<<"y_sw : "<<y_sw<<std::endl;
    FERROR<<"feed_forward : "<<feed_forward<<std::endl;
    return feed_forward;
}

void PID::epsTorqueLimitFunc(float *epsTorque)
{
    static float epsTorque_last = 0;
    *epsTorque = fmax(fmin(*epsTorque, eps_limit_.outmax), eps_limit_.outmin);

    if ((*epsTorque - epsTorque_last) > eps_limit_.step_max) {
        *epsTorque = epsTorque_last + eps_limit_.step_max;
    } else if ((*epsTorque - epsTorque_last) < -eps_limit_.step_max) {
        *epsTorque = epsTorque_last - eps_limit_.step_max;
    }  

    if ((*epsTorque - epsTorque_last) / loop_rate_ > eps_limit_.torque_rate_max) {
        *epsTorque = epsTorque_last + eps_limit_.torque_rate_max * loop_rate_;
    } else if ((*epsTorque - epsTorque_last) / loop_rate_ < -eps_limit_.torque_rate_max) {
        *epsTorque = epsTorque_last - eps_limit_.torque_rate_max * loop_rate_;
    }

    *epsTorque = fmax(fmin(*epsTorque, eps_limit_.outmax), eps_limit_.outmin);

    if (*epsTorque - epsTorque_last > 0.2 || *epsTorque - epsTorque_last < -0.2) {
        FERROR << " deta torque larger than 0.2 "<<std::endl;
        FDEBUG << "epsTorqueLimitFunc : epsTorque , epsTorque_last, d_torque : " 
               << *epsTorque << " , " << epsTorque_last << " , " << *epsTorque - epsTorque_last << std::endl;
    }

    if ((*epsTorque - epsTorque_last) / loop_rate_ > 10 || (*epsTorque - epsTorque_last) / loop_rate_ < -10) {
        FERROR << "torque rate is larger than 10 !"<<std::endl;
        FDEBUG << "epsTorqueLimitFunc : epsTorque_rate : " << (*epsTorque - epsTorque_last) / loop_rate_ << std::endl;
    }

    epsTorque_last = *epsTorque;
}

}
