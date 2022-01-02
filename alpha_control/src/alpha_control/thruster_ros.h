#pragma once

#ifndef ALPHA_CONTROL_THRUSTER_ROS_H
#define ALPHA_CONTROL_THRUSTER_ROS_H

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "Eigen/Dense"

/** @brief Thruster class for managing data
 *
 */
class ThrusterROS{
private:

    //! @brief Public node handler
    ros::NodeHandle m_nh;

    //! @brief Thruster ID
    std::string m_thruster_id;

    //! @brief Thruster topic ID
    std::string m_topic_id;

    //! @brief thruster link id
    std::string m_link_id;

    /** @brief Thruster contribution vector
     *
     * This vector defines a column in control allocation matrix.
     * Each element in the vector describes contribution on
     * vehicle motion of the thruster in each degree of freedom
     */
    Eigen::VectorXf m_contribution_vector;

    //! @brief Thrust publisher
    ros::Publisher m_thrust_publisher;

public:

    //! @brief Default constructor
    ThrusterROS();

    /** @brief Thruster ROS class constructor.
     *
     * This constructor should be used in normal operation.
     * Initializes Thruster ID, Topic ID and contribution vector
     *
     * @param thruster_id
     * @param topic_id
     * @param contribution_vector
     */
    ThrusterROS(std::string thruster_id, std::string topic_id, Eigen::VectorXf contribution_vector);


    void initialize();

    /** @brief Trivial getter for topic id
     *
     * @return #ThrusterROS::m_topic_id
     */
    auto get_topic_id() -> decltype(m_topic_id);

    /** @brief Default Setter for topic id
     *
     * @param topic_id
     */
    void set_topic_id(const decltype(m_topic_id) &topic_id);

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
     * @return #ThrusterROS::m_thruster_id
     */
    auto get_thruster_id() -> decltype(m_thruster_id);

    /** @brief Trivial Setter for topic id
     *
     * @param thruster_id
     */
    void set_thruster_id(const decltype(m_thruster_id)& thruster_id);

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


    //! @brief Generic typedef for shared pointer
    typedef std::shared_ptr<ThrusterROS> Ptr;


    void setpoint(float point);

};


#endif //ALPHA_CONTROL_THRUSTER_ROS_H
