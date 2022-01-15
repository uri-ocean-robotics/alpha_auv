/*
    Program components:
     - <imu_sim.h>
     - imu_sim.cpp
     - Part of the alpha_sensor_sim package in ROS
    
    Author: 
     - Raymond Turrisi <raymond.turrisi@gmail.com>

    Organization: 
     - Smart Ocean Systems Lab

    Licence: 
     - MIT License

    Compilation:
     - catkin_make
    
    System:
     - Ubuntu 20.04 on x86 architecture
    
    Circa:
     - Spring 2022

    Description: 
     - Part of the alpha_sensor_sim package in ROS Noetic. Listens to simulator
      topics and adds noise to the data which represents ideal conditions. Different 
      profiles are loaded per the AHRS/IMU's datasheet that is used. 
*/
#pragma once 

#include "sensor_msgs/Imu.h"
#include "ros/ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "boost/bind.hpp"
#include <chrono>
#include <random>
#include "ros/console.h"
#include "Eigen/Dense"
#include "boost/array.hpp"


#define STATIC_STRING static constexpr const char *
#define STATIC_DOUBLE static constexpr double

namespace conversions {
    STATIC_DOUBLE inches_to_meters = 0.0254;
    STATIC_DOUBLE degs_to_rads = 0.0174533;
    STATIC_DOUBLE grams_to_meters_over_secsec = 9.80665;
    STATIC_DOUBLE milli = 0.001;
    STATIC_DOUBLE micro = 0.000001;
}

namespace ImuSimDict {
    STATIC_STRING CONF_NOISE_TYPE = "noise_type";
    STATIC_STRING CONF_NOISE_TYPE_NO_NOISE = "none";
    STATIC_STRING CONF_NOISE_TYPE_GAUSSIAN_NOISE = "gaussian";
    STATIC_STRING CONF_LINEAR_ACCELERATION_MEAN = "linear_acceleration_mean";
    STATIC_STRING CONF_LINEAR_ACCELERATION_STD = "linear_acceleration_std";
    STATIC_STRING CONF_ANGULAR_VELOCITY_MEAN = "angular_velocity_mean";
    STATIC_STRING CONF_ANGULAR_VELOCITY_STD = "angular_velocity_std";
    STATIC_STRING CONF_ORIENTATION_MEAN = "orientation_mean";
    STATIC_STRING CONF_ORIENTATION_STD = "orientation_std";
    STATIC_STRING CONF_LINK_NAME = "link_name";
    STATIC_STRING CONF_PROFILE = "profile";
    STATIC_STRING CONF_FREQUENCY = "frequency";
    STATIC_STRING CONF_TF_PREFIX = "tf_prefix";
    STATIC_STRING CONF_AXIS_MISALIGNMENT = "axis_misalignment";
    STATIC_STRING CONF_CONSTANT_BIAS = "constant_bias";
    STATIC_STRING CONF_X = "x";
    STATIC_STRING CONF_Y = "y";
    STATIC_STRING CONF_Z = "z";
}

class ImuSim {
private:


    enum NoiseType : uint8_t {
        None = 0,
        Gaussian = 1,
        RandomWalk = 2,
        AxisMisalignment = 3,
        ConstantBias = 4
    };

    double m_rate;

    std::string m_link_name;

    std::string m_tf_prefix;

    std::string m_imu_profile;

    uint8_t m_noise_type;

    ros::NodeHandle m_nh;

    ros::NodeHandle m_pnh;

    ros::Publisher m_imu_sim_data_publisher;

    std::vector<NoiseType> m_noise_profiles;

    static constexpr const char* m_topic_imu = "imu/data";

    static constexpr const char* m_topic_pose = "dynamics/pose";

    static constexpr const char* m_topic_velocity = "dynamics/velocity";

    const std::string m_topic_acceleration = "dynamics/acceleration";

    Eigen::Quaterniond m_axis_misalignment;

    Eigen::Quaterniond m_constant_bias;

    boost::array<double, 9> m_orientation_covariance;

    boost::array<double, 9> m_angular_velocity_covariance;

    boost::array<double, 9> m_linear_acceleration_covariance;

    message_filters::Subscriber<geometry_msgs::PoseStamped> m_pose_subscriber;

    message_filters::Subscriber<geometry_msgs::TwistStamped> m_velocity_subscriber;

    message_filters::Subscriber<geometry_msgs::TwistStamped> m_acceleration_subscriber;

    typedef message_filters::TimeSynchronizer<
        geometry_msgs::PoseStamped,
        geometry_msgs::TwistStamped,
        geometry_msgs::TwistStamped
    > StateSynchronizer;

    std::shared_ptr<StateSynchronizer> m_state_subscriber;

    void f_cb_simulation_state(const geometry_msgs::PoseStamped::ConstPtr &pose,
                               const geometry_msgs::TwistStamped::ConstPtr &vel,
                               const geometry_msgs::TwistStamped::ConstPtr &accel);

    std::mt19937_64 m_generator;

    std::normal_distribution<double> m_linear_acceleration_noise;

    std::normal_distribution<double> m_angular_velocity_noise;

    std::normal_distribution<double> m_orientation_noise;

    void f_generate_parameters();

    void f_apply_constant_bias(sensor_msgs::Imu& msg);

    void f_apply_axis_misalignment(sensor_msgs::Imu& msg);

    void f_apply_noise_density(sensor_msgs::Imu& msg);

    void f_apply_bias_instability(sensor_msgs::Imu& msg);

    void f_apply_random_walk(sensor_msgs::Imu& msg);

    void f_apply_acceleration_bias(sensor_msgs::Imu& msg);

    static void f_msg_to_eigen(const sensor_msgs::Imu& msg,
                               Eigen::Quaterniond &orientation,
                               Eigen::Vector3d &linear_acceleration,
                               Eigen::Vector3d &angular_velocity);

    static void f_compute_covariance_matrix(const std::vector<double>& stddev, boost::array<double, 9> &covariance_out);

public:
    ImuSim();

    void run();
};