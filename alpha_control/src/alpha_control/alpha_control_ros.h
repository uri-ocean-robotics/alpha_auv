#pragma once

#ifndef ALPHA_CONTROL_ALPHA_CONTROL_ROS_H
#define ALPHA_CONTROL_ALPHA_CONTROL_ROS_H

#include "thread"
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "std_msgs/Float32.h"
#include "tf2_eigen/tf2_eigen.h"
#include "nav_msgs/Odometry.h"
#include "dynamic_reconfigure/server.h"
#include "alpha_control/PIDConfig.h"

#include "alpha_control/ControlState.h"
#include "alpha_control/ControlModes.h"
#include "alpha_control/GetControlModes.h"
#include "alpha_control/SetControlPoint.h"

#include "alpha_control.h"

#include "Eigen/Dense"

#include "thruster_ros.h"

#include "boost/shared_ptr.hpp"
#include "boost/thread.hpp"
#include "boost/bind.hpp"


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
    Eigen::MatrixXd m_control_allocation_matrix;

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
    Eigen::VectorXd m_system_state;

    //! @brief Desired state
    Eigen::VectorXd m_desired_state;

    //! @brief Controller frequency
    double m_controller_frequency;

    //! @brief Get control modes ros service server
    ros::ServiceServer m_get_control_modes_server;

    //! @brief Set control point ros service server
    ros::ServiceServer m_set_control_point_server;

    //! @brief Trivial subscriber
    ros::Subscriber m_odometry_subscriber;

    //! @brief Current state publisher
    ros::Publisher m_current_state_publisher;

    //! @brief Desired state subscriber
    ros::Subscriber m_desired_state_subscriber;

    //! @brief Publishes error in the state
    ros::Publisher m_error_state_publisher;

    //! @brief Holder for latest odometry msg
    nav_msgs::Odometry m_odometry_msg;

    //! @brief Control modes stored as ros message
    alpha_control::ControlModes m_control_modes_msg;

    //! @brief Desired state message
    alpha_control::ControlState m_desired_state_msg;

    //! @brief List of control modes
    std::vector<std::string> m_control_modes;

    //! @brief Current control mode
    std::string m_control_mode;

    /** @brief List of all control rules with controlled degrees of freedom
     *
     * m_control_rules[ rule name ] -> degrees of freedom
     * @see STATE_X_INDEX
     * @see STATE_Y_INDEX
     * @see STATE_Z_INDEX
     * @see STATE_ROLL_INDEX
     * @see STATE_PITCH_INDEX
     * @see STATE_YAW_INDEX
     * @see STATE_U_INDEX
     * @see STATE_V_INDEX
     * @see STATE_W_INDEX
     */
    std::map<std::string, std::vector<int>> m_control_rules;

    //! @brief Protects odometry_msg during changes
    boost::recursive_mutex m_odom_lock;

    //! @brief Protects dynamic reconfigure server from dead locks
    boost::recursive_mutex m_config_lock;

    //! @brief Controller worker
    boost::thread m_controller_worker;

    //! @brief Dynamic configure server for PID configuration
    boost::shared_ptr<dynamic_reconfigure::Server<alpha_control::PIDConfig>> m_dynconf_pid_server;

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

    /** @brief Generate thrusters
     *
     */
    void f_generate_thrusters();

    /** @brief Read PID gains
     *
     */
    void f_read_pid_gains();

    /** @brief Reads ros param server and generates control modes
     *
     */
    void f_read_control_modes();

    /** @brief computes state of the system
     *
     */
    bool f_compute_state();

    /** @brief Control Loop
     *
     *
     */
    void f_control_loop();

    /** @brief Amends changes to Dynamic reconfigure server
     *
     * After reading the static configuration file, applies configuration to dynamic reconfigure server.
     */
    void f_amend_dynconf();

    /** @brief Amends the control mode
     *
     * This function checks if request mode is exist or not. Returns false if the operation requested is invalid.
     *
     * @param mode
     * @return
     */
    bool f_amend_control_mode(std::string mode);

    /** @brief Amends the desired stateUpdates
     *
     * Returns false if desired state mode is invalid. See #AlphaControlROS::f_amend_control_mode
     *
     * @param state
     * @return
     */
    bool f_amend_desired_state(const alpha_control::ControlState& state);

    /** @brief Trivial subscriber
     *
     * @param msg
     */
    void f_cb_msg_odometry(const nav_msgs::Odometry::ConstPtr& msg);

    /** @brief Trivial desired state callback
     *
     * @param msg
     */
    void f_cb_srv_desired_state(const alpha_control::ControlState::ConstPtr& msg);

    /** @brief Dynamic reconfigure server callback
     *
     * @param config
     * @param level
     */
    void f_cb_dynconf_pid(alpha_control::PIDConfig &config, uint32_t level);

    /** @brief Trivial ros service server callback for get control modes
     *
     * This service returns configured control modes to ros service client
     *
     * @param req
     * @param resp
     * @return Success of the operation
     */
    bool f_cb_srv_get_control_modes(alpha_control::GetControlModes::Request& req, alpha_control::GetControlModes::Response &resp);

    /** @brief Trivial ros service server callback for set control point
     *
     * @param req
     * @param resp
     * @return Success of the operation
     */
    bool f_cb_srv_set_control_point(alpha_control::SetControlPoint::Request req, alpha_control::SetControlPoint::Response resp);

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
    typedef boost::shared_ptr<AlphaControlROS> Ptr;


};


#endif //ALPHA_CONTROL_ALPHA_CONTROL_ROS_H
