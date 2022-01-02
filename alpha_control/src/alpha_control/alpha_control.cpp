#include "alpha_control.h"

AlphaControl::AlphaControl() {

    m_pid = std::make_shared<MimoPID>();

}

void AlphaControl::set_control_allocation_matrix(const decltype(m_control_allocation_matrix)& matrix) {
    m_control_allocation_matrix = matrix;
}

auto AlphaControl::get_control_allocation_matrix() -> decltype(m_control_allocation_matrix) {
    return m_control_allocation_matrix;
}

auto AlphaControl::get_pid() -> decltype(m_pid) {
    return m_pid;
}

void AlphaControl::set_pid(const MimoPID::Ptr &pid) {
    m_pid = pid;
}
