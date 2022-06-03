#pragma once
#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <cmath>
#include <limits>
#include <Eigen/Dense>
#include "Eigen/Core"
#include <opencv2/core/core.hpp>

#include "common.h"

#include "ego_state/ego_state.h"
#include "mathtool/math_utils.h"

#include "acc_params/acc_params.h"//acc

#define SIM_DEBUG


struct Point_acc {
	Point_acc() {
		x = 0.0;
		y = 0.0;
		theta = 0.0; // deeg
	}

	Point_acc(const double xx, const double yy) {
		x = xx;
		y = yy;
		theta = 0.0;
	}

	Point_acc(const double xx, const double yy, const double angle) {
		x = xx;
		y = yy;
		theta = angle;
	}

	double x;
	double y;
	double theta;
};

struct CipvInfo {
	bool is_valid;
	double dis;
	double v;
};

struct CanInfo {
	double ego_v;
};


struct TrajectoryPoint_acc {
	TrajectoryPoint_acc() {
		x = 0.0;
		y = 0.0;
		theta = 0.0;
		s = 0.0;
		v = 0.0;
	}

	TrajectoryPoint_acc(const double xx, const double yy, const double angle) {
		x = xx;
		y = yy;
		theta = angle;
		s = 0.0;
		v= 0.0;
	}

	TrajectoryPoint_acc(const double xx, const double yy, const double angle, const double ss) {
		x = xx;
		y = yy;
		theta = angle;
		s = ss;
		v = 0.0;
	}

	TrajectoryPoint_acc(const double xx, const double yy, const double angle, const double ss, const double vv) {
		x = xx;
		y = yy;
		theta = angle;
		s = ss;
		v = vv;
	}

	double x;
	double y;
	double theta;
	double s;
	double v;
};


class LqrAcc
{

    using Matrix = Eigen::MatrixXd;
public:
	LqrAcc();
	~LqrAcc();
	void Inital(std::string *file_path);
	// void Inital(double v_set, double ts, double th, double d_offset, double state_size_, double tolerance, double max_num_iteration);
	void LoadData(const TrajectoryPlaningMsg* trajectory,
		          const LocationMsg *ego_pose, 
		          const FrontVehicleMsg* cipv, 
		          const VehicleReportMsg* ego_can,
		          float th);

	double ControllerProc();
private:
    double GetYaw(double x, double y, double z, double w) {
		double yaw(0.0);
		if (!(std::isnan(x) || std::isnan(y) || std::isnan(z) || std::isnan(w))) {
			yaw = std::atan2(2.0 * (w * z + x * y), 1 - 2.0 * (y * y + z * z)) * 180 / M_PI;
		}
    	return yaw;
	}
	int LoadConf(std::string *file_path);
    TrajectoryPoint_acc MatchTrajPoint(const Point_acc& point) const;
    TrajectoryPoint_acc GetPreAimPoint(const TrajectoryPoint_acc& car_matched_point, const double pre_L);
    TrajectoryPoint_acc GetPathPointWithPathS(const double path_s) const ;
    void MatrixInit();
    void UpdateState(const double &d_error, const double &v_error);
    /**
    * @brief Solver for discrete-time linear quadratic problem.
    * @param A The system dynamic matrix
    * @param B The control matrix
    * @param Q The cost matrix for system state
    * @param R The cost matrix for control output
    * @param tolerance The numerical tolerance for solving Discrete
    *        Algebraic Riccati equation (DARE)
    * @param max_num_iteration The maximum iterations for solving ARE
    * @param ptr_K The feedback control matrix (pointer)
    */
    void SolveLQRProblem(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                     const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                     const double tolerance, const uint max_num_iteration,
                     Eigen::MatrixXd *ptr_K);
private:
	CipvInfo cipv_info_;
	CanInfo can_info_;
    std::vector<TrajectoryPoint_acc> plan_trajectory_;//路径
	State ego_state_;//状态
    // double v_set_;//
    double th_;
    // double d_offset_;
    // double tolerance_;
    // double max_num_iteration_;
    double ua_lbound_;
    double ua_ubound_;
    // control time interval
    // double ts_ = 0.0;
    // number of states, include distance error, velocity error
    // const int state_size_ = 2;
    // states matrix
    Eigen::MatrixXd matrix_state_;
    // vehicle states matrix
    Eigen::MatrixXd matrix_a_;
    // vehicle states matrix (discrete-time)
    Eigen::MatrixXd matrix_ad_;
    // control matrix
    Eigen::MatrixXd matrix_b_;
    // control matrix (discrete-time)
    Eigen::MatrixXd matrix_bd_;
    // gain matrix
    Eigen::MatrixXd matrix_k_;
    // state weighting matrix
    Eigen::MatrixXd matrix_q_;
    // control authority weighting matrix
    Eigen::MatrixXd matrix_r_;

    // parameters for lqr solver; number of iterations
    int lqr_max_iteration_ = 0;
    // parameters for lqr solver; threshold for computation
    double lar_eps_ = 0.0;
    AccParams acc_params_;//acc

    double loop_rate_;
    double acc_pid_prev_i_term_;
    double prev_speed_error_;

    


};
