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

#ifndef ALPHA_SIMULATOR_SIMULATOR_H
#define ALPHA_SIMULATOR_SIMULATOR_H

#include "state.hxx"

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/Float64.h"
#include "rosgraph_msgs/Clock.h"
#include "chrono"
#include "thread"

class Simulator {
private:

    double m_simulated_depth;
    uint16_t imu_noise_type = 0;

    uint64_t m_ts; // simulated time stamp

    double m_dt; // seconds

    void iterate(control_commands_t cmd);

    void loop();

    void publish_odometry();

    void publish_velocity();

    void publish_pose();

    void publish_acceleration();

    void publish_clock();

    void main_thruster_cb(const std_msgs::Float64::ConstPtr& msg);

    void horizontal_thruster_cb(const std_msgs::Float64::ConstPtr& msg);

    void vertical_thruster_cb(const std_msgs::Float64::ConstPtr& msg);

    ros::NodeHandle m_nh;

    ros::NodeHandle m_pnh;

    ros::Publisher m_odom_publisher;

    ros::Subscriber m_main_thruster_setpoint;

    ros::Subscriber m_horizontal_thruster_setpoint;

    ros::Subscriber m_vertical_thruster_setpoint;

    ros::Publisher m_acceleration_publisher;

    ros::Publisher m_velocity_publisher;

    ros::Publisher m_pose_publisher;

    ros::Publisher m_clock_publisher;

    std::thread m_loop_thread;

    ros::ServiceClient m_gazebo_set_model_state;

    std::thread m_100hz_thread;

    std::thread m_50hz_thread;

    std::thread m_10hz_thread;

    std::string m_tf_prefix;


public:
    Simulator();

};


#endif //ALPHA_SIMULATOR_SIMULATOR_H
