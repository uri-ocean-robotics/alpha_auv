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

#include "pressure_to_depth_node.h"

PressureToDepthNode::PressureToDepthNode()
{

    m_nh.reset(new ros::NodeHandle(""));
    m_pnh.reset(new ros::NodeHandle("~"));

    m_pnh->param<std::string>("frame_id", m_frame_id, "world");

    m_pressure_subscriber = m_nh->subscribe(
        "pressure", 10, &PressureToDepthNode::f_pressure_callback, this);

    m_depth_publisher = m_nh->advertise
        <mvp_msgs::Float64Stamped>("depth", 10);

    m_pnh->param<double>("water_density", m_fluid_density, 1023.0);

}

void PressureToDepthNode::f_pressure_callback(
    const sensor_msgs::FluidPressureConstPtr &msg)
{

    mvp_msgs::Float64Stamped depth;

    depth.data = msg->fluid_pressure / (m_fluid_density * 9.81);
    depth.header.frame_id = m_frame_id;
    depth.header.stamp = ros::Time::now();

    m_depth_publisher.publish(depth);

}

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "depth_filter");

    PressureToDepthNode d;

    ros::spin();

    return 0;
}