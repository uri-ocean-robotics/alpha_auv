#include "mimo_pid.h"

MimoPID::MimoPID() {

}

Eigen::ArrayXf MimoPID::calculate(const Eigen::ArrayXf &desired, const Eigen::ArrayXf &current) {
    return calculate(desired, current, m_dt);
}

Eigen::ArrayXf MimoPID::calculate(const Eigen::ArrayXf& desired, const Eigen::ArrayXf& current, double dt) {

    Eigen::ArrayXf error = desired - current;

    // Logging errors in a certain timeframe
    m_integral_queue.emplace_back(error * dt);
    if(m_integral_queue.size() > (int)ceil(dt / m_dt_i)) {
        m_integral_queue.pop_front();
    }

    // Proportional term
    Eigen::ArrayXf p = m_kp * error;

    // Integration term
    Eigen::ArrayXf i = Eigen::ArrayXf::Zero(desired.size());

    for(auto& it : m_integral_queue) {
        i += it;
    }

    i = (i > m_max).select(m_max, i);
    i = (i < m_min).select(m_min, i);

    // Derivation term
    if(!m_pe.data()) {
        m_pe = Eigen::VectorXf::Zero(error.size());
    }
    Eigen::ArrayXf d = (error - m_pe) / dt;

    m_pe = error;

    return p + i + d;
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

auto MimoPID::get_max() -> decltype(m_max) {
    return m_max;
}

void MimoPID::set_max(const decltype(m_max) &gain) {
    m_max= gain;
}

auto MimoPID::get_min() -> decltype(m_max) {
    return m_min;
}

void MimoPID::set_min(const decltype(m_min) &gain) {
    m_min= gain;
}
