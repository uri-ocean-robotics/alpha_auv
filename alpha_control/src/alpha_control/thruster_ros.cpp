#include "thruster_ros.h"
#include "exception.hpp"

ThrusterROS::ThrusterROS() {
    m_poly_solver = std::make_shared<PolynomialSolver>();
}

ThrusterROS::ThrusterROS(std::string thruster_id, std::string topic_id, Eigen::VectorXf contribution_vector) :
        m_thruster_id(std::move(thruster_id)),
        m_topic_id(std::move(topic_id)),
        m_contribution_vector(std::move(contribution_vector))
{

    m_thrust_publisher = m_nh.advertise<std_msgs::Float32>(m_topic_id, 10);

    m_poly_solver = std::make_shared<PolynomialSolver>();
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

auto ThrusterROS::get_poly_solver() -> decltype(m_poly_solver) {
    return m_poly_solver;
}

void ThrusterROS::set_poly_solver(decltype(m_poly_solver) solver) {
    m_poly_solver = solver;
}

bool ThrusterROS::request_force(float N) {
    std::vector<std::complex<double>> roots;

    if(!m_poly_solver->solve_for_y(roots, N)) {
        ROS_WARN_STREAM("No feasible setpoint found for force: " << N);
        return false;
    }

    for(const auto& r : roots) {
        if(r.imag() != 0){
            continue;
        }

        if(r.real() >= 1 || r.real() < -1) {
            continue;
        }

        setpoint(r.real());

        break;
    }

    return true;
}