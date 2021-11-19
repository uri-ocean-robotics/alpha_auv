#include "controller_mapper.h"

AlphaController::AlphaController() : m_pnh("~"), m_nh("") {

    m_cmd_vel_subscriber = m_nh.subscribe("controller/cmd_vel", 10, &AlphaController::cmd_vel_callback, this);
    m_set_point_subscriber = m_nh.subscribe("controller/thruster_setpoints", 10, &AlphaController::set_point_callback, this);
    m_cmd_pwm_publisher = m_nh.advertise<geometry_msgs::Vector3Stamped>("controller/cmd_pwm", 10);

}

void AlphaController::cmd_vel_callback(const geometry_msgs::Vector3Stamped &msg) {
    geometry_msgs::Vector3Stamped m;

    m.header = msg.header;
    m.vector.x = msg.vector.x * 500 + 1500;
    m.vector.y = - msg.vector.y * 500 + 1500;
    m.vector.z = msg.vector.z * 500 + 1500;
    m_cmd_pwm_publisher.publish(m);
}

void AlphaController::set_point_callback(const cola2_msgs::Setpoints &msg) {
    geometry_msgs::Vector3Stamped m;
    if(msg.setpoints.size() != 3) {
        return;
    }
    m.header = msg.header;
    m.vector.x = msg.setpoints.at(0) * 500 + 1500;
    m.vector.y = - msg.setpoints.at(1) * 500 + 1500;
    m.vector.z = msg.setpoints.at(2) * 500 + 1500;
    m_cmd_pwm_publisher.publish(m);
}