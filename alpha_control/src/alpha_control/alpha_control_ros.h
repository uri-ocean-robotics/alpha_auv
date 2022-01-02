#pragma once

#ifndef ALPHA_CONTROL_ALPHA_CONTROL_ROS_H
#define ALPHA_CONTROL_ALPHA_CONTROL_ROS_H

#include "thread"
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "std_msgs/Float32.h"
#include "tf2_eigen/tf2_eigen.h"
#include "nav_msgs/Odometry.h"
#include "alpha_control/ControlState.h"

#include "alpha_control.h"

#include "Eigen/Dense"

#include "vector"
#include "memory"
#include "thruster_ros.h"


#define CONTROL_ALLOCATION_MATRIX_GENERATOR_TYPE_TF "tf"
#define CONTROL_ALLOCATION_MATRIX_GENERATOR_TYPE_USER "user"


extern std::mutex g_odom_lock;

/** @brief ROS wrapper for AlphaControl
 *
 *  This package wraps alpha control class and utilizes its
 *  functionality.
 *
 *  @see AlphaControl
 */
class AlphaControlROS {
private:
    /*! @brief Generator Type enum class
     *
     */
    enum class GeneratorType {
        TF,
        USER,
        UNKNOWN
    };

    //! @brief Public node handler
    ros::NodeHandle m_nh;

    //! @brief Private node handler
    ros::NodeHandle m_pnh;

    //! @brief Thruster list
    std::vector<ThrusterROS::Ptr> m_thrusters;

    /**! @brief Control Allocation Matrix
     *
     *  Control allocation matrix is generated from individual
     *  configurations of the thrusters.
     */
    Eigen::MatrixXf m_control_allocation_matrix;

    //! @brief Control allocation matrix generator type
    GeneratorType m_generator_type;

    //! @brief Center of gravity link id
    std::string m_cg_link_id;

    //! @brief World link id
    std::string m_world_link_id;

    //! @brief Transform buffer for TF2
    tf2_ros::Buffer m_transform_buffer;

    //! @brief Transform listener for TF2
    tf2_ros::TransformListener m_transform_listener;

    //! @brief Transform prefix
    std::string m_tf_prefix;

    //! @brief Alpha Control object
    AlphaControl::Ptr m_alpha_control;

    //! @brief System state
    Eigen::VectorXf m_system_state;

    //! @brief Desired state
    Eigen::VectorXf m_desired_state;

    //! @brief control rate
    std::shared_ptr<ros::Rate> m_control_rate;

    //! @brief Trivial subscriber
    ros::Subscriber m_odometry_subscriber;

    //! @brief Current state publisher
    ros::Publisher m_current_state_publisher;

    //! @brief Desired state subscriber
    ros::Subscriber m_desired_state_subscriber;

    //! @brief Holder for latest odometry msg
    nav_msgs::Odometry m_odometry_msg;

    //! @brief Desired state message
    alpha_control::ControlState m_desired_state_msg;

    /** @brief Generates control allocation matrix from transform tree
     *
     *  This method is called if generator_type is 'tf'
     */
    void f_generate_control_allocation_from_tf();

    /** @brief Generates control allocation matrix from user input
     *
     *  This method is called if generator_type is 'user'
     */
    void f_generate_control_allocation_from_user();

    /**! @brief Generates control allocation matrix
     *
     */
    void f_generate_control_allocation_matrix();

    /** @brief Read PID gains
     *
     */
    void f_read_pid_gains();


    /** @brief computes state of the system
     *
     */
    bool f_compute_state();

    /** @brief Control Loop
     *
     *
     */
    void f_control_loop();

    /** @brief Trivial subscriber
     *
     * @param msg
     */
    void f_odometry_cb(const nav_msgs::Odometry::ConstPtr& msg);

    void f_desired_state_cb(const alpha_control::ControlState::ConstPtr& msg);


    //! @brief Controller worker
    std::thread m_controller_worker;

public:

    /** @brief Default constructor
     *
     */
    AlphaControlROS();

    /** @brief Initializer for Alpha Control ROS
     *
     * This function initializes control allocation matrix and
     *
     */
    void initialize();

    //! @brief Generic typedef for shared pointer
    typedef std::shared_ptr<AlphaControlROS> Ptr;



};


#endif //ALPHA_CONTROL_ALPHA_CONTROL_ROS_H
