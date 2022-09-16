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

#include "depth_filter_node.h"

DepthFilterNode::DepthFilterNode() {
    m_nh.reset(new ros::NodeHandle(""));
    m_pnh.reset(new ros::NodeHandle("~"));


    double A, B, H, Q, R, P, x;

    m_pnh->param<double>("A", A, 1.0);
    m_pnh->param<double>("B", B, 0.0);
    m_pnh->param<double>("H", H, 1.0);
    m_pnh->param<double>("Q", Q, 1e-4);
    m_pnh->param<double>("R", R, 1e-4);
    m_pnh->param<double>("P", P, 1.0);
    m_pnh->param<double>("x", x, 0.0);


    m_pnh->param<std::string>("frame_id", m_frame_id, "world");

    m_filter.reset(new UnoLqe(x, A, B, H, Q, R, P));

    m_depth_subscriber = m_nh->subscribe("depth", 10, &DepthFilterNode::f_depth_callback, this);
    m_depth_publisher = m_nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("depth_filtered", 10);

}

void DepthFilterNode::f_depth_callback(const mvp_msgs::Float64StampedConstPtr &msg) {

    m_filter->predict();
    m_filter->update(msg->data);

    double x, p;
    m_filter->state(&x, &p);

    geometry_msgs::PoseWithCovarianceStamped out;

    out.header = msg->header;
    out.header.frame_id = m_frame_id;
    out.pose.pose.position.x = 0.0;
    out.pose.pose.position.y = 0.0;
    out.pose.pose.position.z = -x;

    out.pose.covariance[6 * 2 + 2] = p;

    m_depth_publisher.publish(out);

}


int main(int argc, char* argv[]) {

    ros::init(argc, argv, "depth_filter");

    DepthFilterNode d;

    ros::spin();

    return 0;
}