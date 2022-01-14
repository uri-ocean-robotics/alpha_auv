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


namespace mf = message_filters;

typedef struct imu_state {
    imu_state();
    std::string frame_id;
    ros::Time recent_time;

    //orientation
    double roll;
    double pitch;
    double yaw;
    double x_quat; 
    double y_quat;
    double w_quat;
    double z_quat;

    //angular velocity
    double p;
    double q;
    double r;

    //linear acceleration
    double u_dot;
    double v_dot;
    double w_dot;
} imu_state;

class ImuSim {
    private:
        struct conversions {
            static constexpr double inches_to_meters = 0.0254;
            static constexpr double degs_to_rads = 0.0174533;
            static constexpr double grams_to_meters_over_secsec = 9.80665;
            static constexpr double milli = 0.001;
            static constexpr double micro = 0.000001;
        };

        const std::string no_noise = "NO_NOISE";
        const std::string gaussian_noise = "GAUSSIAN_NOISE";
        u_int32_t m_loop_rate;

        std::string m_imu_profile = "NOT_SELECTED";
        std::string m_noise_type = "NOT_SELECTED";
        imu_state m_imu_state;

        //ROS handles
        ros::NodeHandle m_node_handle;
        ros::NodeHandle m_pnode_handle;

        ////associated topic names
        const std::string m_imu_pubto_topic_data = "imu/data";
        const std::string m_dynamics_listo_pose_topic = "dynamics/pose";
        const std::string m_dynamics_listo_velocity_topic = "dynamics/velocity";
        const std::string m_dynamics_listo_acceleration_topic = "dynamics/acceleration";

        ////publishers
        ros::Publisher m_imu_sim_data_publisher;
        
        ////time syncronized subscribers ROS subscribers. See message_filters and TimeSynchronizers
        mf::Subscriber<geometry_msgs::PoseStamped> m_dynamics_pose_subscriber;
        mf::Subscriber<geometry_msgs::TwistStamped> m_dynamics_velocity_subscriber;
        mf::Subscriber<geometry_msgs::TwistStamped> m_dynamics_acceleration_subscriber;
        mf::TimeSynchronizer<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped, geometry_msgs::TwistStamped> m_state_subscriber;

        
        ////ROS types and utilities
        ros::Time m_recent_time;
        tf2::Quaternion m_quat;

        ////subscriber callback
        void f_extract_dynamics_state(const geometry_msgs::PoseStamped::ConstPtr &pose, 
                                    const geometry_msgs::TwistStamped::ConstPtr &vel, 
                                    const geometry_msgs::TwistStamped::ConstPtr &accel);


        sensor_msgs::Imu f_get_msg();

        //Gaussian noise implementation
        //std::default_random_engine m_generator;
        std::mt19937_64 m_generator;
        std::normal_distribution<double> m_imu_lin_accel_distribution;
        std::normal_distribution<double> m_imu_angvel_distribution;
        std::normal_distribution<double> m_imu_orientation_distribution;        

        //the function that ends up applying noise
        using msg_fn = sensor_msgs::Imu (ImuSim::*)();

        msg_fn f_noise_applicator;

        //Individual noise application methods
        sensor_msgs::Imu f_add_no_noise();

        sensor_msgs::Imu f_add_gaussian_noise();
        
        void f_load_ros_params();
    public:
        ImuSim();

        ~ImuSim();
        
        void step();

        void run();
};