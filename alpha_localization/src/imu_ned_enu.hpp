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

#pragma once

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "memory"

class IMUNedEnu {
    private:
        ros::NodeHandlePtr m_nh;

        ros::NodeHandlePtr m_pnh;

        ros::Publisher m_imu_out;

        ros::Subscriber m_imu_in;

        std::string m_frame_id;

        void f_imu_callback(const sensor_msgs::ImuConstPtr& msg);
    public:
        IMUNedEnu();

};