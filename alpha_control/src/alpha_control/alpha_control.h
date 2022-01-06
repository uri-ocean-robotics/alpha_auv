#pragma once

#ifndef ALPHA_CONTROL_ALPHA_CONTROL_H
#define ALPHA_CONTROL_ALPHA_CONTROL_H

#include "memory"
#include "Eigen/Dense"
#include "mimo_pid.h"
#include "osqp++.h"


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

    //! @brief Desired State
    Eigen::VectorXd m_desired_state;

    //! @brief Error State
    Eigen::VectorXd m_error_state;

    bool f_calculate_pid(Eigen::VectorXd &u, double dt);

    bool f_optimize_thrust(Eigen::VectorXd &t, Eigen::VectorXd u);

    std::vector<int> m_controlled_freedoms;

    Eigen::ArrayXd f_error_function(Eigen::ArrayXd desired, Eigen::ArrayXd current);



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

    bool calculate_needed_forces(Eigen::VectorXd &t, float dt);

    void set_controlled_freedoms(decltype(m_controlled_freedoms) f);

    auto get_state_error() -> decltype(m_error_state);

};


#endif //ALPHA_CONTROL_ALPHA_CONTROL_H
