#include "mimo_pid.h"
#include "exception.hpp"

MimoPID::MimoPID() : m_error_function(nullptr) , m_dt_i(10){

}

bool MimoPID::calculate(Eigen::VectorXd& u, const Eigen::ArrayXd& desired, const Eigen::ArrayXd& current, double dt) {

    if(m_error_function == nullptr) {
        throw control_exception("error function is not defined for MIMO pid.");
    }

    Eigen::ArrayXd error = m_error_function(desired, current);

    // Logging errors in a certain timeframe
    m_integral_queue.emplace_back(error * dt);
    if(m_integral_queue.size() > (int)ceil(m_dt_i / dt)) {
        m_integral_queue.pop_front();
    }

    // Proportional term
    Eigen::ArrayXd p = m_kp * error;

    // Integration term
    Eigen::ArrayXd i = Eigen::ArrayXd::Zero(desired.size());

    for(auto& it : m_integral_queue) {
        i += it;
    }

    i = (i > m_i_max).select(m_i_max, i);
    i = (i < m_i_min).select(m_i_min, i);

    // Derivation term
    if(!m_pe.data()) {
        m_pe = Eigen::VectorXd::Zero(error.size());
        return false;
    }
    Eigen::ArrayXd d = (error - m_pe) / dt;

    m_pe = error;

    u = p + i + d;

    return true;
}

auto MimoPID::get_kp() -> decltype(m_kp) {
    return m_kp;
}

void MimoPID::set_kp(const decltype(m_kp) &gain) {
    m_kp = gain;
}

auto MimoPID::get_ki() -> decltype(m_ki) {
    return m_ki;
}

void MimoPID::set_ki(const decltype(m_ki) &gain) {
    m_ki = gain;
}

auto MimoPID::get_kd() -> decltype(m_kd) {
    return m_kd;
}

void MimoPID::set_kd(const decltype(m_kd) &gain) {
    m_kd = gain;
}

auto MimoPID::get_dt() const -> decltype(m_dt) {
    return m_dt;
}

void MimoPID::set_dt(const decltype(m_dt) &gain) {
    m_dt = gain;
}

auto MimoPID::get_dt_i() const -> decltype(m_dt_i) {
    return m_dt_i;
}

void MimoPID::set_dt_i(const decltype(m_dt_i) &gain) {
    m_dt_i = gain;
}

auto MimoPID::get_i_max() -> decltype(m_i_max) {
    return m_i_max;
}

void MimoPID::set_i_max(const decltype(m_i_max) &gain) {
    m_i_max= gain;
}

auto MimoPID::get_i_min() -> decltype(m_i_max) {
    return m_i_min;
}

void MimoPID::set_i_min(const decltype(m_i_min) &gain) {
    m_i_min= gain;
}

auto MimoPID::get_error_function() -> decltype(m_error_function) {
    return m_error_function;
}

void MimoPID::set_error_function(const decltype(m_error_function) &func) {
    m_error_function = func;
}
