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

#include <ros/ros.h>
#include "control_handle.hpp"

#include<cassert>
#include<fstream>
typedef ns_control::ControlHandle ControlHandle;

int main(int argc, char **argv)
{
  std::ifstream infile;
  infile.open("/home/shuangji/FSD_ALGO/src/ros/control/controller/result_min.csv");
  assert(infile.is_open());
std::string line,cell;
  std::vector<double> x_point;
  std::vector<double> y_point;
  std::vector<double> yaw_point;
  std::vector<double> vel_point;
  std::cout<<"doneeee"<<std::endl;
  while (getline(infile, line))
  {
   std::istringstream iss(line);
    int column = 0;
     // 逐个单元格读取每一行
        while (std::getline(iss, cell, ',')) {
            double value = std::stod(cell); // 将单元格的字符串转换为double类型

            // 根据列号将数据存储到相应的向量中
            switch (column) {
                case 0:
                  x_point.push_back(value);
                    break;
                case 1:
                  y_point.push_back(value);
                    break;
                case 2:
                  yaw_point.push_back(value);
                    break;
                case 3:
                    vel_point.push_back(value);
                    break;
            }

            column++;
        }
    }
  

  infile.close();
  ns_control::csvtraj trackingpath;

  for (int i = 0; i < x_point.size(); ++i)
  {
    ns_control::CsvPoint trackingpoint;
    trackingpoint.pointx = x_point[i];
    trackingpoint.pointy = y_point[i];
    trackingpoint.pointyaw = yaw_point[i];
    trackingpoint.vel_flying = vel_point[i];
    trackingpath.push_back(trackingpoint);
  }
  ros::init(argc, argv, "control");
  ros::NodeHandle nodeHandle("~");
  ControlHandle myControlHandle(nodeHandle);
  ros::Rate loop_rate(myControlHandle.getNodeRate());
  while (ros::ok())
  {

    myControlHandle.run(trackingpath);

    ros::spinOnce();   // Keeps node alive basically
    loop_rate.sleep(); // Sleep for loop_rate
  }
  return 0;
}
