#pragma once

#ifndef ALPHA_CONTROL_THRUSTER_ROS_H
#define ALPHA_CONTROL_THRUSTER_ROS_H

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "Eigen/Dense"
#include "polynomial_solver.h"

#include "boost/shared_ptr.hpp"

/** @brief Thruster class for managing data
 *
 */
class ThrusterROS{
private:

    //! @brief Public node handler
    ros::NodeHandle m_nh;

    //! @brief Thruster ID
    std::string m_id;

    //! @brief Thrust command topic ID
    std::string m_thrust_command_topic_id;

    //! @brief Thruster force topic id
    std::string m_thrust_force_topic_id;

    //! @brief thruster link id
    std::string m_link_id;

    /** @brief Thruster contribution vector
     *
     * This vector defines a column in control allocation matrix.
     * Each element in the vector describes contribution on
     * vehicle motion of the thruster in each degree of freedom
     */
    Eigen::VectorXd m_contribution_vector;

    //! @brief Thrust publisher
    ros::Publisher m_thrust_publisher;

    //! @brief Thrust force publisher
    ros::Publisher m_force_publisher;

    //! @brief Polynomial solver
    PolynomialSolver::Ptr m_poly_solver;

public:

    //! @brief Default constructor
    ThrusterROS();

    /** @brief Thruster ROS class constructor.
     *
     * This constructor should be used in normal operation.
     * Initializes Thruster ID, Topic ID and contribution vector
     *
     * @param id
     * @param topic_id
     * @param contribution_vector
     */
    ThrusterROS(std::string id, std::string topic_id, Eigen::VectorXd contribution_vector);

    /** @brief Initializes publishers and subscribers
     *
     */
    void initialize();

    /** @brief Trivial getter for topic id
     *
     * @return #ThrusterROS::m_thrust_command_topic_id
     */
    auto get_thrust_command_topic_id() -> decltype(m_thrust_command_topic_id);

    /** @brief Default Setter for topic id
     *
     * @param topic_id
     */
    void set_thrust_command_topic_id(const decltype(m_thrust_command_topic_id) &topic_id);

    /** @brief Trivial getter for force topic id
     *
     * @return #ThrusterROS::m_thrust_force_topic_id
     */
    auto get_thrust_force_topic_id() -> decltype(m_thrust_force_topic_id);

    /** @brief Default Setter force for topic id
     *
     * @param topic_id
     */
    void set_thrust_force_topic_id(const decltype(m_thrust_force_topic_id) &topic_id);

    /** @brief Trivial getter for link id
     *
     * @return #ThrusterROS::m_link_id
     */
    auto get_link_id() -> decltype(m_link_id);

    /** @brief Trivial Setter for link id
     *
     * @param link_id
     */
    void set_link_id(const decltype(m_link_id)& link_id);


    /** @brief Trivial getter for thruster id
     *
     * @return #ThrusterROS::m_id
     */
    auto get_id() -> decltype(m_id);

    /** @brief Trivial Setter for topic id
     *
     * @param thruster_id
     */
    void set_id(const decltype(m_id)& thruster_id);

    /** @brief Trivial getter for contribution vector
     *
     * @return #ThrusterROS::m_contribution_vector
     */
    auto get_contribution_vector() -> decltype(m_contribution_vector);

    /** @brief Trivial Setter for contribution vector
     *
     * @param contribution Contribution vector for the thruster
     */
    void set_contribution_vector(const decltype(m_contribution_vector )& contribution_vector);

    /** @brief Trivial getter for polynomial solver
     *
     * @return #ThrusterROS::m_poly_solver
     */
    auto get_poly_solver() -> decltype(m_poly_solver);

    /** @brief Trivial setter for polynomial solver
     *
     * @param solver
     */
    void set_poly_solver(decltype(m_poly_solver) solver);

    //! @brief Generic typedef for shared pointer
    typedef boost::shared_ptr<ThrusterROS> Ptr;

    /** @brief Publish thruster command
     *
     * Thuster command should be between -1 and 1
     *
     * @param cmd
     */
    void command(float cmd);

    /** @brief Request force from thruster
     *
     * This method gets input \p N as Newton and applies it to a polynomial solver
     * that is defined with #PolynomialSolver::m_coeff.
     *
     * @param N force as newton
     * @return true if polynomial is solved, false if polynomial isn't solved.
     */
    bool request_force(double N);
};


#endif //ALPHA_CONTROL_THRUSTER_ROS_H
