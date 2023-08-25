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

#include "Solver/mpc_ki_solver.h"
#include "ros/ros.h"
#include <cmath>

namespace ns_control {
    
void MPCKI_SOLVER::solve() {
    ROS_INFO_STREAM("begin solve");
    if (trajectory_.empty()) {
        control_command_.throttle.data       = static_cast<float>(-1.0);//类型转换
        control_command_.steering_angle.data = 0.0;
        ROS_INFO_STREAM("trajectory empty");
        return;
    }

    double desire_vel = control_param_.desire_vel;

mpc_solve(desire_vel);

  


 
}
bool MPCKI_SOLVER::mpc_solve(double desire_speed)
{
OsqpEigen:: Solver solver;
    solver.settings() -> setWarmStart(true);
   temp(solver,desire_speed);
    solver.solve();
    Eigen::VectorXd QPSolution;
    QPSolution = solver.getSolution(); 
    double v_temp = QPSolution[0];
    double delta_temp = QPSolution[1];    
    control_command_.steering_angle.data=delta_temp;
    control_command_.throttle.data=v_temp;




}
bool MPCKI_SOLVER::temp(OsqpEigen::Solver& solver,double desire_speed)
{
  
   if (initMat(solver,desire_speed)) 
   {
        if (!solver.initSolver()) return 1;
    }
    else {
        cout << "initilize QP solver failed" << endl;
        return 1;
    }
}

bool MPCKI_SOLVER::initMat(OsqpEigen::Solver& solver,double desire_speed)
{ 
  phi_ref=trajectory_[1].yaw+state_.yaw;
  delta_ref=;
   A<<1,0,-ts*desire_speed*sin(phi_ref),
     0,1,ts*desire_speed*cos(phi_ref),
     0,0,1;
   
   B<<ts*cos(phi_ref),0,
     ts*sin(phi_ref),0,
     ts*tan(delta_ref)/wheelbase,ts*desire_speed/(wheelbase*pow(cos(delta_ref),2));
  
   C<<A,A*A,A*A*A,A*A*A*A,A*A*A*A*A;
   
   Eigen::MatrixXd zero(3,2);
    zero<<0,0,
          0,0,
          0,0;
    M<<B,zero,zero,zero,zero,
       A*B,B,zero,zero,zero,
       A*A*B,A*B,B,zero,zero,
       A*A*A*B,A*A*B,A*B,B,zero,
       A*A*A*A*B,A*A*A*B,A*A*B,A*B,B;
    
    Eigen::MatrixXd zeros1(3,3);
    zeros1<<0,0,0,
            0,0,0,
            0,0,0;
    Eigen::MatrixXd Q(3,3);
      Q<<1,0,0,
         0,1,0,
         0,0,1;         
    Q_bar<<Q,zeros1,zeros1,zeros1,zeros1,
           zeros1,Q,zeros1,zeros1,zeros1,
           zeros1,zeros1,Q,zeros1,zeros1,
           zeros1,zeros1,zeros1,Q,zeros1,
           zeros1,zeros1,zeros1,zeros1,Q;
    Eigen::MatrixXd zeros2(2,2);
    zeros2<<0,0,
            0,0;
    Eigen::MatrixXd R(2,2);
    R<<1,0,
       0,1;
    R_bar<<R,zeros2,zeros2,zeros2,zeros2,
           zeros2,R,zeros2,zeros2,zeros2,
           zeros2,zeros2,R,zeros2,zeros2,
           zeros2,zeros2,zeros2,R,zeros2,
           zeros2,zeros2,zeros2,zeros2,R;    
solver.data()->setNumberOfVariables(10);
    solver.data()->setNumberOfConstraints(10);    
    H=M.transpose()*Q_bar*M + R_bar;   

    Hessian=H.sparseView(1, 1e-20);
  g=delta_state.transpose()*C.transpose()*Q_bar*M; 
  gradient=g.transpose();

  linearMatrix.resize(10,10);
  temp2<<1,0,0,0,0,0,0,0,0,0,
         0,1,0,0,0,0,0,0,0,0,
         0,0,1,0,0,0,0,0,0,0,
         0,0,0,1,0,0,0,0,0,0,
         0,0,0,0,1,0,0,0,0,0,
         0,0,0,0,0,1,0,0,0,0,
         0,0,0,0,0,0,1,0,0,0,
         0,0,0,0,0,0,0,1,0,0,
         0,0,0,0,0,0,0,0,1,0,
         0,0,0,0,0,0,0,0,0,1;
 linearMatrix=temp2.sparseView(1, 1e-20);
   
    lowerBound.resize(10);
    upperBound.resize(10);
    lowerBound << 0,-0.69,0,-0.69,0,-0.69,0,-0.69,0,-0.69;
    upperBound << 10,0.69,10,0.69,10,0.69,10,0.69,10,0.69;
    
    if (!solver.data()->setHessianMatrix(Hessian)) return false;
    if (!solver.data()->setGradient(gradient)) return false;
    if (!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return false;
    if (!solver.data()->setLowerBound(lowerBound)) return false;
    if (!solver.data()->setUpperBound(upperBound)) return false;
return true;
}

}