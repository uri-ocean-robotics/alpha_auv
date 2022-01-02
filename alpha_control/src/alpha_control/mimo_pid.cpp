#include "mimo_pid.h"

MimoPID::MimoPID() {

}

Eigen::ArrayXd MimoPID::calculate(const Eigen::ArrayXd &desired, const Eigen::ArrayXd &current) {
    return calculate(desired, current, m_dt);
}

Eigen::ArrayXd MimoPID::calculate(const Eigen::ArrayXd& desired, const Eigen::ArrayXd& current, double dt) {

    Eigen::ArrayXd error = desired - current;

    // Logging errors in a certain timeframe
    m_integral_queue.emplace_back(error * dt);
    if((double)m_integral_queue.size() > ceil(dt / m_dt_i)) {
        m_integral_queue.pop_front();
    }

    // Proportional term
    auto p = m_kp * error;

    // Integration term
    Eigen::ArrayXd i = Eigen::ArrayXd::Zero(desired.size());

    for(auto& it : m_integral_queue) {
        i += it;
    }

    // Derivation term
    auto d = (error - m_pe) / dt;

    Eigen::ArrayXd output = p + i + d;

    output = (output > m_max).select(m_max, output);
    output = (output < m_min).select(m_min, output);

    m_pe = error;

    return output;
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

