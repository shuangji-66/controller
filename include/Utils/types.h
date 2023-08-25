#ifndef TYPE_H
#define TYPE_H

#include <iostream>
#include "opencv2/opencv.hpp"

// ros package
#include "std_msgs/Float64MultiArray.h"

// custom messages
#include "fsd_common_msgs/CarState.h"
#include "fsd_common_msgs/ControlCommand.h"
#include "fsd_common_msgs/Map.h"

// STL
#include <cmath>
#include <vector>
#include <map>
namespace ns_control
{
    struct VehicleState
    {
        double x;
        double y;
        double yaw;
        double v;
        double r;
        double a;
        double w;
        double Delta;
        double D;
        double vx;
        double vy;
        double ax;
        double ay;

        VehicleState(fsd_common_msgs::CarState state, fsd_common_msgs::ControlCommand cmd)
        {
            x = state.car_state.x;
            y = state.car_state.y;
            yaw = state.car_state.theta;
            v = std::hypot(state.car_state_dt.car_state_dt.x, state.car_state_dt.car_state_dt.y);
            r = state.car_state_dt.car_state_dt.theta;
            vx = state.car_state_dt.car_state_dt.x;
            vy = state.car_state_dt.car_state_dt.y;
            ax = state.car_state_dt.car_state_a.x;
            ay = state.car_state_dt.car_state_a.y;
            a = std::hypot(state.car_state_dt.car_state_a.x, state.car_state_dt.car_state_a.y);
            w = state.car_state_dt.car_state_a.theta;

            D = cmd.throttle.data;
            Delta = cmd.steering_angle.data;
        }

        VehicleState()
        {
        }
    };

    struct TrajectoryPoint
    {
        cv::Point2f pts;
        double yaw;
        double curvature;
        double velocity;
        double r;
        double acc;
    };
    struct CsvPoint
    {

        double pointx;
        double pointy;
        double pointyaw;
        double vel_flying;
    };
    typedef std::vector<CsvPoint>csvtraj;

    
    typedef std::vector<TrajectoryPoint> Trajectory;
    struct LateralControlError
    {
        double lateral_error;      // 横向误差
        double heading_error;      // 转向误差
        double lateral_error_rate; // 横向误差速率
        double heading_error_rate; // 转向误差速度
    };
    typedef std::shared_ptr<LateralControlError> LateralControlErrorPtr;
    struct LongitudinalError
    {
        double station_error;
        double station_rate_error;
    };
    typedef std::shared_ptr<LongitudinalError> LongitudinalErrorPtr;

} // namespace waypoint_follower

#endif