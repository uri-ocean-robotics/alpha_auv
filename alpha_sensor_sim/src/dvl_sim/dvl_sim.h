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

namespace conversions {
    STATIC_DOUBLE inches_to_meters = 0.0254;
    STATIC_DOUBLE degs_to_rads = 0.0174533;
    STATIC_DOUBLE grams_to_meters_over_secsec = 9.80665;
    STATIC_DOUBLE milli = 0.001;
    STATIC_DOUBLE micro = 0.000001;
}

namespace DvlSimDict {
    STATIC_STRING CONF_NOISE_TYPE = "noise_type";
    STATIC_STRING CONF_NOISE_TYPES = "noise_types";
    STATIC_STRING CONF_NOISE_NO_NOISE = "none";
    STATIC_STRING CONF_NOISE_GAUSSIAN_NOISE = "gaussian";
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

class DvlSim {
private:
    enum NoiseType : uint8_t {
        None = 0,
        Gaussian = 1,
        RandomWalk = 2,
        AxisMisalignment = 3,
        ConstantBias = 4
    };

    typedef struct vehicle_state_t {
        vehicle_state_t();
        //pose
        //double x;
        //double y;
        double depth;
        double dist_to_seafloor;

        double p;
        double q;
        double r;

        //velocities
        double u;
        double v;
        double w;
        
        double p_dot;
        double q_dot;
        double r_dot;
    } vehicle_state_t;

    vehicle_state_t vehicle_state;

    double m_rate;

    double m_max_range = 50;

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

    std::shared_ptr<std::mt19937_64> m_generator;

    std::normal_distribution<double> m_angular_velocity_noise;

    std::normal_distribution<double> m_orientation_noise;

    void f_generate_parameters();

    void f_apply_constant_bias(alpha_sensor_sim::Transducer& msg);

    void f_apply_axis_misalignment(alpha_sensor_sim::Transducer& msg);

    void f_apply_noise_density(alpha_sensor_sim::Transducer& msg);

    void f_apply_bias_instability(alpha_sensor_sim::Transducer& msg);

    void f_apply_random_walk(alpha_sensor_sim::Transducer& msg);

public:
    DvlSim();

    void run() const;
};