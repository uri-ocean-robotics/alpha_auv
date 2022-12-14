/*
    This file is part of ALPHA AUV project.

    This project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the project.  If not, see <https://www.gnu.org/licenses/>.

    Author: Emir Cem Gezer
    Email: emircem@uri.edu;emircem.gezer@gmail.com
    Year: 2022

    Copyright (C) 2022 Smart Ocean Systems Laboratory
*/

#include "Eigen/Dense"
#include "iostream"
#include "cstdio"
#include "imu_ned_enu.hpp"

IMUNedEnu::IMUNedEnu() {

    m_nh.reset(new ros::NodeHandle());

    m_imu_in =  m_nh->subscribe("imu_in/data", 10, &IMUNedEnu::f_imu_callback, this);

    m_imu_out = m_nh->advertise<sensor_msgs::Imu>("imu_out/data", 10);
}

void IMUNedEnu::f_imu_callback(const sensor_msgs::ImuConstPtr& msg) {

    Eigen::Quaterniond i{
        msg->orientation.w,
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z
    };

    // auto euler = i.toRotationMatrix().eulerAngles(0, 1, 2);

    // Eigen::Quaterniond o = i;
    // o = Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitX())
    //     * Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY())
    //     * Eigen::AngleAxisd(-euler.z(), Eigen::Vector3d::UnitZ());

    i = i * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    sensor_msgs::Imu m = *msg;
    m.orientation.w = i.w();
    m.orientation.x = i.y();
    m.orientation.y = i.x();
    m.orientation.z = -i.z();

    m_imu_out.publish(m);

}

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "imu_ned_to_enu");

    IMUNedEnu i;

    ros::spin();

    return 0;
}
