/*
    Formula Student Driverless Project (FSD-Project).
    Copyright (c) 2021:
     - chentairan <tairanchen@bitfsd.cn>

    FSD-Project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    FSD-Project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with FSD-Project.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "Solver/stanley_solver.h"
#include "ros/ros.h"
#include <cmath>
// #include "gp.h"
// #include "gp_utils.h"
#include <vector>
#include <Eigen/Dense>
namespace ns_control
{
  // 两点之间的距离
  double Stanley_Solver::PointDistanceSquare(const ns_control::CsvPoint &point, const double x,
                                             const double y)
  {

    double dx = point.pointx - x;
    double dy = point.pointy - y;
    return dx * dx + dy * dy;
  }
  int Stanley_Solver::findnearestindex(const ns_control::csvtraj &path, VehicleState state_)
  {
    int index = 0;
    double d_min = PointDistanceSquare(path.front(), state_.x, state_.y);
    for (int i = 1; i < path.size(); ++i)
    {

      double d_temp = PointDistanceSquare(path[i], state_.x, state_.y);
      if (d_temp < d_min)
      {
        d_min = d_temp;
        index = i;
      }
    }
  }
  // using namespace libgp;
  void Stanley_Solver::solve(ns_control::csvtraj trackingpath_)
  {

    // ROS_INFO_STREAM("begin solve");
    // if (trajectory_.empty())
    // {
    //   control_command_.throttle.data = static_cast<float>(-1.0); // 类型转换
    //   control_command_.steering_angle.data = 0.0;
    //   ROS_INFO_STREAM("trajectory empty");
    //   return;
    // }
    //  int n = 40, m = 10;
    // double tss = 0, error, f, y;

    // // 初始化 Gaussian process 对象
    // GaussianProcess gp(2, "CovSum ( CovSEiso, CovNoise)");

    // // 初始化超参数向量
    // Eigen::VectorXd params(gp.covf().get_param_dim());
    // params << 0.0, 0.0, -2.0;

    // // 设置核函数的参数
    // gp.covf().set_loghyper(params);

    // // 添加训练样本
    // for (int i = 0; i < n; ++i) {
    //   double x[] = {drand48() * 4 - 2, drand48() * 4 - 2};
    //   y = Utils::hill(x[0], x[1]) + Utils::randn() * 0.1;
    //   gp.add_pattern(x, y);
    // }

    // std::vector<double> y_plot;
    // std::vector<double> f_plot;

    // // 计算总体平方误差并记录结果
    // for (int i = 0; i < m; ++i) {
    //   double x[] = {drand48() * 4 - 2, drand48() * 4 - 2};
    //   f = gp.f(x);
    //   y = Utils::hill(x[0], x[1]);
    //   y_plot.push_back(y);
    //   f_plot.push_back(f);
    //   error = f - y;
    //   tss += error * error;
    // }

    // std::cout << "mse = " << tss / m << std::endl;
    double desire_vel = control_param_.desire_vel;
    int index = findnearestindex(trackingpath_, state_);

    std::cout << "find!!!!!!!!!" << index << std::endl;
    std::vector<double> ref_vel;
    std::vector<double> xpoint;
    std::vector<double> ypoint;
    std::vector<double> yaw;
    if (index < trackingpath_.size() - 40)
    {

      for (int i = 0; i < 40; ++i)
      {
        int new_index = i + index;
        ref_vel.push_back(trackingpath_[new_index].vel_flying + 2);
        xpoint.push_back(trackingpath_[new_index].pointx);
        ypoint.push_back(trackingpath_[new_index].pointy);
        yaw.push_back(trackingpath_[new_index].pointyaw);
      }
    }

    else
    {
      for (int i = 0; i < trackingpath_.size() - index; ++i)
      {
        ref_vel.push_back(trackingpath_[i + index].vel_flying);
        xpoint.push_back(trackingpath_[i + index].pointx);
        ypoint.push_back(trackingpath_[i + index].pointy);
        yaw.push_back(trackingpath_[i + index].pointyaw);
      }
      for (int i = 0; i < 40 - (trackingpath_.size() - index); ++i)
      {
        ref_vel.push_back(trackingpath_[i].vel_flying);
        xpoint.push_back(trackingpath_[i].pointx);
        ypoint.push_back(trackingpath_[i].pointy);
        yaw.push_back(trackingpath_[i].pointyaw);
      }
    }
    Trajectory traj_new;
    TrajectoryPoint traj_point;
    for (int i = 0; i < ref_vel.size(); ++i)
    {
      traj_point.pts.x = xpoint[i];
      traj_point.pts.y = ypoint[i];
      traj_point.yaw = yaw[i];
      traj_point.velocity = ref_vel[i];
      traj_point.acc = 0;
      traj_point.curvature = 0;
      traj_point.r = 0;
      traj_new.push_back(traj_point);
    }

    { // Steering Control
      ComputeControlCmd(traj_new,state_);
      std::cout
          << "steering:  " << control_command_.steering_angle.data << std::endl;
    }
    { // Speed Controller
      const double vel = state_.v;
      PIDController speedpid(0.2, 0.2, 0.1);
      control_command_.throttle.data = speedpid.Control(desire_vel - vel, 0.1);
    }

    // // 使用 GNU Plot 绘制数据图
    // FILE *gnuplot = popen("gnuplot -persist", "w");
    // if (gnuplot != nullptr) {
    //   // 绘制 y_plot 数据
    //   fprintf(gnuplot, "plot '-' with lines title 'y_plot'\n");
    //   for (size_t i = 0; i < y_plot.size(); ++i) {
    //     fprintf(gnuplot, "%f\n", y_plot[i]);
    //   }
    //   fprintf(gnuplot, "e\n");

    //   // 绘制 f_plot 数据
    //   fprintf(gnuplot, "replot '-' with lines title 'f_plot'\n");
    //   for (size_t i = 0; i < f_plot.size(); ++i) {
    //     fprintf(gnuplot, "%f\n", f_plot[i]);
    //   }
    //   fprintf(gnuplot, "e\n");

    //   // 关闭 GNU Plot 进程
    //   pclose(gnuplot);
    // } else {
    //   std::cerr << "无法启动 GNU Plot 进程" << std::endl;
    // }
  }
  void Stanley_Solver::ComputeControlCmd(const Trajectory &traj,const VehicleState &vehicle_state)
  {
    double e_y_;
    double e_theta_;
    double veh_x = vehicle_state.x;
    double veh_y = vehicle_state.y;
    double veh_head = vehicle_state.yaw;
    double veh_v = vehicle_state.v + 0.001;

    ComputeLateralErrors(traj,veh_x, veh_y, veh_head, e_y_, e_theta_);
    double k_y_ = 0.8;
    double tra_steer = e_theta_ + atan2((k_y_ * e_y_), veh_v);
    if (tra_steer > 1)
      tra_steer = 1;
    else if (tra_steer < -1)
      tra_steer = -1;
    else
    {
    }
    control_command_.steering_angle.data = tra_steer;
  }
  void Stanley_Solver::ComputeLateralErrors(const Trajectory &traj,const double x, const double y, const double theta,
                                            double &e_y, double &e_theta)

  {

    auto trajectory_points = traj[3];

    double dx = (trajectory_points.pts.x - x);
    double dy = (trajectory_points.pts.y - y);

    e_y = dy * cos(trajectory_points.yaw) - dx * sin(trajectory_points.yaw);

    e_theta = trajectory_points.yaw - theta;

    if (e_theta > M_PI)
    {
      e_theta = e_theta - M_PI * 2;
    }
    if (e_theta < -M_PI)
    {
      e_theta = e_theta + M_PI * 2;
    }
  }
}
