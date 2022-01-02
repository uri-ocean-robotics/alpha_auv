#include "thruster_ros.h"
#include "exception.hpp"

ThrusterROS::ThrusterROS() = default;

ThrusterROS::ThrusterROS(std::string thruster_id, std::string topic_id, Eigen::VectorXf contribution_vector) :
        m_thruster_id(std::move(thruster_id)),
        m_topic_id(std::move(topic_id)),
        m_contribution_vector(std::move(contribution_vector))
{

    m_thrust_publisher = m_nh.advertise<std_msgs::Float32>(m_topic_id, 10);

}

auto ThrusterROS::get_topic_id() -> decltype(m_topic_id) {
    return m_topic_id;
}

void ThrusterROS::set_topic_id(const decltype(m_topic_id) &topic_id) {
    m_topic_id = topic_id;
}

auto ThrusterROS::get_thruster_id() -> decltype(m_thruster_id) {
    return m_thruster_id;
}

void ThrusterROS::set_thruster_id(const decltype(m_thruster_id)& thruster_id) {
    m_thruster_id = thruster_id;
}

auto ThrusterROS::get_contribution_vector() -> decltype(m_contribution_vector) {
    return m_contribution_vector;
}

void ThrusterROS::set_contribution_vector(const decltype(m_contribution_vector)& contribution_vector) {
    m_contribution_vector = contribution_vector;
}

void ThrusterROS::initialize() {

    if(!m_topic_id.empty()) {
        m_thrust_publisher = m_nh.advertise<std_msgs::Float32>(m_topic_id, 100);
    } else {
        throw control_ros_exception("empty topic name");
    }

}

auto ThrusterROS::get_link_id() -> decltype(m_link_id) {
    return m_link_id;
}

void ThrusterROS::set_link_id(const decltype(m_link_id)& link_id) {
    m_link_id = link_id;
}

void ThrusterROS::setpoint(float point) {
    std_msgs::Float32 msg;
    msg.data = point;
    m_thrust_publisher.publish(msg);
}