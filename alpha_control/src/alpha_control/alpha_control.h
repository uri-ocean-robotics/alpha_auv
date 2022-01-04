#pragma once

#ifndef ALPHA_CONTROL_ALPHA_CONTROL_H
#define ALPHA_CONTROL_ALPHA_CONTROL_H

#include "memory"
#include "Eigen/Dense"
#include "mimo_pid.h"
#include "osqp++.h"

#define THRUST_LIMIT_NEWTON 40

/** @brief AlphaControl Class
 *
 */
class AlphaControl {
private:

    //! @brief Control allocation matrix
    Eigen::MatrixXf m_control_allocation_matrix;

    //! @brief MIMO PID Controller
    MimoPID::Ptr m_pid;

    //! @brief System State
    Eigen::VectorXf m_system_state;

    //! @brief Desired State
    Eigen::VectorXf m_desired_state;

    void f_calculate_pid(Eigen::VectorXf &u, double dt);

    bool f_optimize_thrust(Eigen::VectorXf &t, Eigen::VectorXf u);

    std::vector<int> m_controlled_freedoms;

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

    /** @brief Trivial getter for desired state
     *
     * @return #AlphaControl::m_desired_state
     */
    auto get_desired_state() -> decltype(m_desired_state);

    /** @brief Trivial setter for desired state
     *
     * @param desired_state
     */
    void set_desired_state(const decltype(m_desired_state) &desired_state);

    Eigen::VectorXf calculate_setpoints(float dt);

    void set_controlled_freedoms(decltype(m_controlled_freedoms) f);

};


#endif //ALPHA_CONTROL_ALPHA_CONTROL_H
