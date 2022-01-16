/*
    Program components:
     - <dvl_sim.h>
     - dvl_sim.cpp
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
     - 
*/
#pragma once 

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
#include "alpha_sensor_sim/Transducer.h"

#define STATIC_STRING static constexpr const char *
#define STATIC_DOUBLE static constexpr double

namespace convert {
    STATIC_DOUBLE percent_to_decimal = 0.01;
}

namespace DvlSimDict {
    STATIC_STRING CONF_DVL_MAX_RANGE = "dvl_max_range";
    STATIC_STRING CONF_SEAFLOOR_DEPTH = "seafloor_depth";
    STATIC_STRING CONF_NOISE_TYPE = "noise_type";
    STATIC_STRING CONF_NOISE_TYPES = "noise_types";
    STATIC_STRING CONF_NOISE_NO_NOISE = "none";
    STATIC_STRING CONF_NOISE_UNIFORM_NOISE = "uniform";
    STATIC_STRING CONF_NOISE_PERR_VELOCITY = "velocity_average_percent_err";
    STATIC_STRING CONF_NOISE_PERR_DISTANCE = "distance_average_percent_err";
    STATIC_STRING CONF_NOISE_AXIS_MISALIGNMENT = "axis_misalignment";
    STATIC_STRING CONF_NOISE_CONSTANT_BIAS = "constant_bias";
    STATIC_STRING CONF_NOISE_RANDOM_WALK = "random_walk";
    STATIC_STRING CONF_LINK_NAME = "link_name";
    STATIC_STRING CONF_PROFILE = "profile";
    STATIC_STRING CONF_FREQUENCY = "frequency";
    STATIC_STRING CONF_TF_PREFIX = "tf_prefix";
    STATIC_STRING CONF_X = "x";
    STATIC_STRING CONF_Y = "y";
    STATIC_STRING CONF_Z = "z";
}

typedef struct dvl_state_t {
    dvl_state_t();
    // dvl specific
    double depth;
    double dist_to_seafloor;
    double dvl_distance;
    double dvl_vel;

    //// from pose
    Eigen::Vector3d orientation;

    //// from velocities
    Eigen::Vector3d lin_velocity;
    Eigen::Vector3d ang_velocity;

} dvl_state_t;

class DvlSim {
private:
    enum NoiseType : uint8_t {
        None = 0,
        Uniform = 1,
        RandomWalk = 2,
        AxisMisalignment = 3,
        ConstantBias = 4
    };

    dvl_state_t dvl_state;

    double m_rate;

    double m_artificial_seafloor_depth;

    double m_max_range;

    std::vector<uint8_t> m_noise_types;

    std::string m_link_name;

    std::string m_tf_prefix;

    std::string m_dvl_profile;

    uint8_t m_noise_type;

    ros::NodeHandle m_nh;

    ros::NodeHandle m_pnh;

    ros::Publisher m_dvl_sim_data_publisher;

    std::vector<NoiseType> m_noise_profiles;

    static constexpr const char* m_topic_dvl = "dvl/data";

    static constexpr const char* m_topic_pose = "dynamics/pose";

    static constexpr const char* m_topic_velocity = "dynamics/velocity";

    Eigen::Quaterniond m_axis_misalignment;

    Eigen::Quaterniond m_constant_bias;

    message_filters::Subscriber<geometry_msgs::PoseStamped> m_pose_subscriber;

    message_filters::Subscriber<geometry_msgs::TwistStamped> m_velocity_subscriber;

    typedef message_filters::TimeSynchronizer<
        geometry_msgs::PoseStamped,
        geometry_msgs::TwistStamped
        > StateSynchronizer;

    std::shared_ptr<StateSynchronizer> m_state_subscriber;

    void f_cb_simulation_state(const geometry_msgs::PoseStamped::ConstPtr &pose,
                               const geometry_msgs::TwistStamped::ConstPtr &vel);

    std::mt19937_64 m_generator;

    std::uniform_real_distribution<double> m_dvl_velocity_noise;
    std::uniform_real_distribution<double> m_dvl_distance_noise;

    void f_generate_parameters();

    double f_get_dvl_velocity();

    double f_get_dvl_dist();

    void f_apply_constant_bias(alpha_sensor_sim::Transducer& msg);

    void f_apply_axis_misalignment(alpha_sensor_sim::Transducer& msg);

    void f_apply_noise_density(alpha_sensor_sim::Transducer& msg);

    void f_apply_bias_instability(alpha_sensor_sim::Transducer& msg);

    void f_apply_random_walk(alpha_sensor_sim::Transducer& msg);

public:
    DvlSim();

    void run() const;
};