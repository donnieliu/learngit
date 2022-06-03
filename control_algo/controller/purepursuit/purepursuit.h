#pragma once
#include <iostream>
#include <cmath>
#include <array>
#include <algorithm>
#include <iostream>
#include <vector>

#include "common.h"

#include "mathtool/math_utils.h"
#include "ego_state/ego_state.h"
#include "vehicle_params/vehicle_params.h"

#define SIM_DEBUG

#define DEG_TO_RAD .017453292519943296
struct Point {
	Point() {
		x = 0.0;
		y = 0.0;
		theta = 0.0; // deeg
	}

	Point(const double xx, const double yy) {
		x = xx;
		y = yy;
		theta = 0.0;
	}

	Point(const double xx, const double yy, const double angle) {
		x = xx;
		y = yy;
		theta = angle;
	}

	double x;
	double y;
	double theta;
};

struct TrajectoryPoint {
	TrajectoryPoint() {
		x = 0.0;
		y = 0.0;
		theta = 0.0;
		s = 0.0;
		v = 0.0;
	}

	TrajectoryPoint(const double xx, const double yy, const double angle) {
		x = xx;
		y = yy;
		theta = angle;
		s = 0.0;
		v= 0.0;
	}

	TrajectoryPoint(const double xx, const double yy, const double angle, const double ss) {
		x = xx;
		y = yy;
		theta = angle;
		s = ss;
		v = 0.0;
	}

	TrajectoryPoint(const double xx, const double yy, const double angle, const double ss, const double vv) {
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


class PurePursuit
{
public:
	PurePursuit();
	~PurePursuit();
	int Init(std::string *file_path);
	int LoadData(const TrajectoryPlaningMsg* trajectory,
		const LocationMsg *ego_pose, const VehicleReportMsg* ego_can);
	int ControllerProc();
	double GetTargetAcceleration();
	double GetTargetSteerAngle();
private:
	int LoadConf(std::string *file_path);
	int CheckIn();
	int LonControllerProc(const TrajectoryPoint& aim_point, double lat_error, double angle_error);
	int LatControllerProc(const Point& ego_car_position, const TrajectoryPoint& aim_point,
		double lat_error, double angle_error);
	int OdomTransLocal(const Point& ego_car_odom, const Point& target_odom, Point& target_local) const;
	int LocalTransOdom(const Point &ego_car_odom, const Point &target_local, Point& target_odom) const;
	Point EstimateFuturePosition(const double t) const;
	TrajectoryPoint MatchTrajPoint(const Point& point) const;
	TrajectoryPoint GetPreAimPoint(const TrajectoryPoint& car_matched_point,
		const double pre_L);
	double GetParkLength();
	inline double GetYaw(double x, double y, double z, double w) {
		double yaw(0.0);
		if (!(std::isnan(x) || std::isnan(y) || std::isnan(z) || std::isnan(w))) {
			yaw = std::atan2(2.0 * (w * z + x * y), 1 - 2.0 * (y * y + z * z)) * 180 / M_PI;
		}
    	return yaw;
	}

	std::pair<double, double> ComputePositionProjection(const double x, const double y,
		const TrajectoryPoint& p) const ;
	std::pair<double, double> GetTrackError(const Point reference_point_odom,
		const TrajectoryPoint& traj_point) const;
	TrajectoryPoint GetPathPointWithPathS(const double path_s) const ;

private:
	std::vector<TrajectoryPoint> plan_trajectory_;
	State ego_state_;
	double output_a_;
	double output_steer_angle_;
	double kv_;
	double kl_;
	double dl_;
	double pre_l_min_;
	double pre_l_max_;
	double park_length_;
	double car_speed_;
  	double steer_step_max_;

	std::array<double, 10> deta_output_;
	std::pair<double, double> track_error_record_;

	VehicleParams ego_params_;

};
