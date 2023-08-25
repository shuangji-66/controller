#pragma once
#include "Solver/solver_base.h"
#include "Eigen/Core"
#include<math.h>
#include<fstream>
#include<iomanip>
#include<memory>
#include<string>
#include"Solver/mpc_osqp_solver.h"
using Matrix = Eigen::MatrixXd;
namespace ns_control
{

    class MPC_MINE_Solver : public Solver
    {
    public:
        void solve();

    protected:
        void LoadControlConf();
        void init();
        bool ComputeControlCommand(const VehicleState &localization,
                                   const Trajectory &planning_published_trajectory,
                                   fsd_common_msgs::ControlCommand &cmd);
        void UpdateState(const VehicleState &vehicle_state);

        void UpdateMatrix(const VehicleState &vehicle_state);
        void ComputeLateralErrors(const double x, const double y, const double theta, 
                                  const double linear_v, const double angular_v,
                                  const double linear_a,
                                  LateralControlErrorPtr &lat_con_err);
        void ComputeLongitudinalErrors(const double x,
                                       const double y, const double v,
                                       const double theta,
                                       LongitudinalErrorPtr &lon_err);
        void SolveLinearMPC(const Matrix &matrix_a, const Matrix &matrix_b,
                            const Matrix &matrix_c, const Matrix &matrix_q,
                            const Matrix &matrix_r, const Matrix &matrix_lower,
                            const Matrix &matrix_upper,
                            const Matrix &matrix_initial_state,
                            const std::vector<Matrix> &reference, const double eps,
                            const int max_iter, std::vector<Matrix> *control);
    private:
        // the following parameters are vehicle physics related.
        // control time interval
        double ts_ = 0.0;
        // corner stiffness; front
        double cf_ = 0.0;
        // corner stiffness; rear
        double cr_ = 0.0;
        // distance between front and rear wheel center
        double wheelbase_ = 0.0;
        // mass of the vehicle
        double mass_ = 0.0;
        // distance from front wheel center to COM
        double lf_ = 0.0;
        // distance from rear wheel center to COM
        double lr_ = 0.0;
        // rotational inertia
        double iz_ = 0.0;
        // the ratio between the turn of the steering wheel and the turn of the wheels
        double steer_ratio_ = 0.0;
        // the maximum turn of steer
        double steer_single_direction_max_degree_ = 0.0;

        // number of states without previews, includes
        // lateral error, lateral error rate, heading error, heading error rate
        const int basic_state_size_ = 4;
        // vehicle state matrix
        Eigen::MatrixXd matrix_a_;
        // vehicle state matrix (discrete-time)
        Eigen::MatrixXd matrix_ad_;
        // control matrix
        Eigen::MatrixXd matrix_b_;
        // control matrix (discrete-time)
        Eigen::MatrixXd matrix_bd_;
        Eigen::MatrixXd matrix_c_;
        Eigen::MatrixXd matrix_cd_;
        // gain matrix
        Eigen::MatrixXd matrix_k_;
        // control authority weighting matrix
        Eigen::MatrixXd matrix_r_;
        // state weighting matrix
        Eigen::MatrixXd matrix_q_;
        // updated state weighting matrix
        Eigen::MatrixXd matrix_q_updated_;
        Eigen::MatrixXd matrix_r_updated_;
        // vehicle state matrix coefficients
        Eigen::MatrixXd matrix_a_coeff_;
        // 4 by 1 matrix; state matrix
        Eigen::MatrixXd matrix_state_;

        // parameters for lqr solver; number of iterations
        int mpc_max_iteration_ = 1500;
        // parameters for lqr solver; threshold for computation
        double mpc_eps_ = 0.01;
        const int horizen_ = 10;
        // Look-ahead controller
        bool enable_look_ahead_back_control_ = false;

        // for compute the differential valute to estimate acceleration/lon_jerk
        double previous_lateral_acceleration_ = 0.0;

        double previous_heading_rate_ = 0.0;
        double previous_ref_heading_rate_ = 0.0;

        double previous_heading_acceleration_ = 0.0;
        double previous_ref_heading_acceleration_ = 0.0;

        // for logging purpose
        std::ofstream steer_log_file_;

        const std::string name_;

        double query_relative_time_;

        double pre_steer_angle_ = 0.0;

        double pre_steering_position_ = 0.0;

        double minimum_speed_protection_ = 0.1;

        double current_trajectory_timestamp_ = -1.0;

        double init_vehicle_x_ = 0.0;

        double init_vehicle_y_ = 0.0;

        double init_vehicle_heading_ = 0.0;

        double low_speed_bound_ = 0.0;

        double low_speed_window_ = 0.0;

        double driving_orientation_ = 0.0;

        double steer_offset_ = 0.0;

        // added
        double ref_curv_;
        int control_ = 1;
        double max_acceleration_ = 5.0;
        double max_deceleration_ = -5.0;
    };

}; // namespace ns_control