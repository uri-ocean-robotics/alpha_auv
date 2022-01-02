#pragma once

#ifndef ALPHA_CONTROL_ALPHA_CONTROL_H
#define ALPHA_CONTROL_ALPHA_CONTROL_H

#include "memory"

#include "Eigen/Dense"
#include "mimo_pid.h"

/** @brief AlphaControl Class
 *
 */
class AlphaControl {
private:

    //! @brief Control allocation matrix
    Eigen::MatrixXd m_control_allocation_matrix;

    //! @brief MIMO PID Controller
    MimoPID::Ptr m_pid;

    //! @brief System State
    Eigen::VectorXd m_system_state;

public:
    /** @brief Alpha Control default constructor
     *
     */
    AlphaControl();


    /** @brief Trivial Setter for control allocation matrix
     *
     * @param matrix
     */
    void set_control_allocation_matrix(const decltype(m_control_allocation_matrix)& matrix);


    /** @brief Trivial getter for thruster id
     *
     * @return #AlphaControl::m_control_allocation_matrix
     */
    auto get_control_allocation_matrix() -> decltype(m_control_allocation_matrix);

    //! @brief Standard shared pointer type
    typedef std::shared_ptr<AlphaControl> Ptr;

    /** @brief Trivial getter for pid controller
     *
     * @return #AlphaControl::m_pid
     */
    auto get_pid() -> decltype(m_pid);

    /** @brief Trivial setter for pid controller
     *
     * @param pid
     */
    void set_pid(const decltype(m_pid) &pid);

    /** @brief Trivial getter for system state
     *
     * @return #AlphaControl::m_system_state
     */
    auto get_system_state() -> decltype(m_system_state);

    /** @brief Trivial setter for system state
     *
     * @param system_state
     */
    void set_system_state(const decltype(m_system_state) &system_state);

};


#endif //ALPHA_CONTROL_ALPHA_CONTROL_H
