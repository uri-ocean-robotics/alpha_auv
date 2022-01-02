#pragma once

#ifndef ALPHA_CONTROL_MIMO_PID_H
#define ALPHA_CONTROL_MIMO_PID_H

#include "Eigen/Dense"
#include "memory"
#include "deque"

class MimoPID {
private:

    /** @brief Delta Time
     *
     * This variable holds the time difference between each iteration
     */
    double m_dt;

    /** @brief Integral window time
     *
     * This variable hold integral time window
     */
    double m_dt_i;

    /**
     * @brief Maximum value
     *
     * This variable holds the maximum value of a gain
     */
    Eigen::ArrayXd m_max;

    /**
     * @brief Minimum value
     *
     * This variable holds the minimum value of a gain
     */
    Eigen::ArrayXd m_min;

    //! @brief Proportional gain
    Eigen::ArrayXd m_kp;

    //! @brief Derivation gain
    Eigen::ArrayXd m_kd;

    //! @brief Integration gain
    Eigen::ArrayXd m_ki;

    //! @brief Error from the previous iteration
    Eigen::ArrayXd m_pe;

    //! @brief Integral queue
    std::deque<Eigen::ArrayXd> m_integral_queue;

public:

    MimoPID();

    /** @brief Calculates PID gain with given desired and current state
     *
     * @param desired   A vector defines the desired state of the system
     * @param current   A vector defines the current state of the system
     * @return          A vector with control gains
     */
    Eigen::ArrayXd calculate(const Eigen::ArrayXd& desired, const Eigen::ArrayXd& current);

    /** @brief Calculates PID gain with given desired and current state
     *
     * @param desired   A vector defines the desired state of the system
     * @param current   A vector defines the current state of the system
     * @param dt        Time difference between readings
     * @return          A vector with control gains
     */
    Eigen::ArrayXd calculate(const Eigen::ArrayXd& desired, const Eigen::ArrayXd& current, double dt);

    //! @brief Generic shared pointer
    typedef std::shared_ptr<MimoPID> Ptr;

    //! @brief Default getter for proportional gain
    auto get_kp() -> decltype(m_kp);

    /*! @brief Default setter for proportional gain
     *
     * @param gain
     */
    void set_kp(const decltype(m_kp) &gain);

    //! @brief Default getter for integral gain
    auto get_ki() -> decltype(m_ki);

    /*! @brief Default setter for integral gain
     *
     * @param gain
     */
    void set_ki(const decltype(m_ki) &gain);

    //! @brief Default getter for derivative gain
    auto get_kd() ->  decltype(m_kd);

    /*! @brief Default setter for derivative gain
     *
     * @param gain
     */
    void set_kd(const decltype(m_kd) &gain);

    //! @brief Default getter for delta time
    auto get_dt() const -> decltype(m_dt);

    /*! @brief Default setter for delta time
     *
     * @param gain
     */
    void set_dt(const decltype(m_dt) &gain);

    //! @brief Default getter for integral time window
    auto get_dt_i() const -> decltype(m_dt_i);

    /*! @brief Default setter for integral time window
     *
     * @param gain
     */
    void set_dt_i(const decltype(m_dt_i) &gain);

};


#endif //ALPHA_CONTROL_MIMO_PID_H
