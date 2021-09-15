#include "controller.h"
#include "Eigen/Geometry"

Controller::Controller() : m_nh() , m_pnh("~"){
    m_nh.subscribe("odom", 100, &Controller::odom_callback, this);
}

void Controller::odom_callback(const nav_msgs::Odometry::ConstPtr &msg) {
    m_odom = *msg;
}

void Controller::iterate() {
    Eigen::Quaternion<double> q;
    q.w() = m_odom.pose.pose.orientation.w;
    q.x() = m_odom.pose.pose.orientation.x;
    q.y() = m_odom.pose.pose.orientation.y;
    q.z() = m_odom.pose.pose.orientation.z;

    auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);

}