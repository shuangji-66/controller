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

#pragma once
#include "Solver/solver_base.h"
#include "Solver/pid_controller.h"
namespace ns_control
{

  class Stanley_Solver : public Solver
  {
  public:
    void solve(ns_control::csvtraj trackingpath_);
    void ComputeControlCmd(const VehicleState &vehicle_state);
    void ComputeLateralErrors(const double x, const double y, const double theta,
                              double &e_y, double &e_theta);
                              
  };

}; // namespace ns_control
