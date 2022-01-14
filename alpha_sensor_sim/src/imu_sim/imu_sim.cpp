/*
    Program components:
     - ImuSim.h
     - <ImuSim.cpp>
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

#include "imu_sim.h"

#define IMU_RAND_SEED std::chrono::system_clock::now().time_since_epoch().count()

namespace mf = message_filters;

imu_state::imu_state() {
    frame_id = "";
    recent_time;
    p = -1;
    q = -1;
    r = -1;
    u_dot = -1;
    v_dot = -1;
    w_dot = -1;
    roll = -1;
    pitch = -1;
    yaw = -1;
    x_quat = -1; 
    y_quat = -1;
    w_quat = -1;
    z_quat = -1;
};

////subscriber callback
void ImuSim::f_extract_dynamics_state(const geometry_msgs::PoseStamped::ConstPtr &pose, 
                            const geometry_msgs::TwistStamped::ConstPtr &vel, 
                            const geometry_msgs::TwistStamped::ConstPtr &accel) {
    //extract only relevant information from those topics, keep them in the standard vehicle state struct
    this->m_imu_state.frame_id = pose->header.frame_id;
    this->m_imu_state.recent_time = pose->header.stamp;
    this->m_imu_state.x_quat = pose->pose.orientation.x;
    this->m_imu_state.y_quat = pose->pose.orientation.y;
    this->m_imu_state.z_quat = pose->pose.orientation.z;
    this->m_imu_state.w_quat = pose->pose.orientation.w;
    double roll, pitch, yaw;
    tf2::fromMsg(pose->pose.orientation, this->m_quat);
    tf2::Matrix3x3(this->m_quat).getRPY(roll, pitch, yaw);
    this->m_imu_state.roll = roll;
    this->m_imu_state.pitch = pitch;
    this->m_imu_state.yaw = yaw;

    this->m_imu_state.p = vel->twist.angular.x;
    this->m_imu_state.q = vel->twist.angular.y;
    this->m_imu_state.r = vel->twist.angular.z;

    this->m_imu_state.u_dot = accel->twist.linear.x;
    this->m_imu_state.v_dot = accel->twist.linear.y;
    this->m_imu_state.w_dot = accel->twist.linear.z;
};


sensor_msgs::Imu ImuSim::f_get_msg() {
    return (this->*f_noise_applicator)();
};

sensor_msgs::Imu ImuSim::f_add_no_noise() {
    
    sensor_msgs::Imu msg;

    msg.header.stamp = this->m_imu_state.recent_time;
    msg.header.frame_id = this->m_imu_state.frame_id;

    msg.orientation.w = this->m_imu_state.w_quat;
    msg.orientation.x = this->m_imu_state.x_quat;
    msg.orientation.y = this->m_imu_state.y_quat;
    msg.orientation.z = this->m_imu_state.z_quat;

    msg.angular_velocity.x = this->m_imu_state.p;
    msg.angular_velocity.y = this->m_imu_state.q;
    msg.angular_velocity.z = this->m_imu_state.r;

    msg.linear_acceleration.x = this->m_imu_state.u_dot;
    msg.linear_acceleration.y = this->m_imu_state.v_dot;
    msg.linear_acceleration.z = this->m_imu_state.w_dot;

    //todo: determine how covariance is to be implemented and if its needed here
    for(int i : {0,1,2,3,4,5,6,7,8}) {
        msg.orientation_covariance[i] = -1.0;
        msg.angular_velocity_covariance[i] = -1.0;
        msg.linear_acceleration_covariance[i] = -1.0;
    }
    return msg;
}

sensor_msgs::Imu ImuSim::f_add_gaussian_noise() {
    sensor_msgs::Imu msg;

    msg.header.stamp = this->m_imu_state.recent_time;
    msg.header.frame_id = this->m_imu_state.frame_id;

    this->m_quat.setRPY(this->m_imu_state.roll+this->m_imu_orientation_distribution(m_generator), 
                        this->m_imu_state.pitch+this->m_imu_orientation_distribution(m_generator),
                        this->m_imu_state.yaw+this->m_imu_orientation_distribution(m_generator));

    msg.orientation.w = this->m_quat.w();
    msg.orientation.x = this->m_quat.x();
    msg.orientation.y = this->m_quat.y();
    msg.orientation.z = this->m_quat.z();

    msg.angular_velocity.x = this->m_imu_state.p+this->m_imu_angvel_distribution(m_generator);
    msg.angular_velocity.y = this->m_imu_state.q+this->m_imu_angvel_distribution(m_generator);
    msg.angular_velocity.z = this->m_imu_state.r+this->m_imu_angvel_distribution(m_generator);

    msg.linear_acceleration.x = this->m_imu_state.u_dot+this->m_imu_lin_accel_distribution(m_generator);
    msg.linear_acceleration.y = this->m_imu_state.v_dot+this->m_imu_lin_accel_distribution(m_generator);
    msg.linear_acceleration.z = this->m_imu_state.w_dot+this->m_imu_lin_accel_distribution(m_generator);

    //todo: determine how covariance is to be implemented and if its needed here
    for(int i : {0,1,2,3,4,5,6,7,8}) {
        msg.orientation_covariance[i] = -1.0;
        msg.angular_velocity_covariance[i] = -1.0;
        msg.linear_acceleration_covariance[i] = -1.0;
    }
    return msg;
}
        
void ImuSim::f_load_ros_params() {

    //get imu profile
    std::string profile;
    m_pnode_handle.param<std::string>("imu_profile", profile, "NO_PROFILE");
    this->m_imu_profile = profile;

    //get noise type
    std::string noise_type;
    m_pnode_handle.param<std::string>("imu_noise_type", noise_type, "NO_TYPE");
    this->m_noise_type = noise_type;

    //get node frequency
    int loop_rate_temp;
    m_pnode_handle.param<int>("imu_frequency", loop_rate_temp, 100);
    this->m_loop_rate = loop_rate_temp;

    //declare temporary local variables
    double imu_lin_acc_mean;
    double imu_lin_acc_std;
    double imu_angvel_mean;
    double imu_angvel_std;
    double imu_orientation_mean;
    double imu_orientation_std;

    //get and set imu profile parameters
    m_pnode_handle.param<double>("imu_lin_acc_mean", imu_lin_acc_mean, -100);
    m_pnode_handle.param<double>("imu_lin_acc_std", imu_lin_acc_std, -100);
    m_pnode_handle.param<double>("imu_angvel_mean", imu_angvel_mean, -100);
    m_pnode_handle.param<double>("imu_angvel_std", imu_angvel_std, -100);
    m_pnode_handle.param<double>("imu_orientation_mean", imu_orientation_mean, -100);
    m_pnode_handle.param<double>("imu_orientation_std", imu_orientation_std, -100);

    //convert to expected units in ROS per the description in the imu.yaml file
    double sqrt_freq = sqrt(static_cast<double>(this->m_loop_rate));

    imu_lin_acc_mean *= conversions::grams_to_meters_over_secsec;
    imu_lin_acc_std *= conversions::grams_to_meters_over_secsec*conversions::micro*sqrt_freq;
    imu_angvel_mean *= conversions::degs_to_rads;
    imu_angvel_std *= conversions::degs_to_rads*conversions::milli*sqrt_freq;
    imu_orientation_mean *= conversions::degs_to_rads;
    imu_orientation_std *= conversions::degs_to_rads;
    
    //assign distribution types
    this->m_imu_lin_accel_distribution =  std::normal_distribution<double>(imu_lin_acc_mean, imu_lin_acc_std);
    this->m_imu_angvel_distribution =  std::normal_distribution<double>(imu_angvel_mean, imu_angvel_std);
    this->m_imu_orientation_distribution =  std::normal_distribution<double>(imu_orientation_mean, imu_orientation_std);
}
   
void ImuSim::step() {
    m_imu_sim_data_publisher.publish(this->f_get_msg());
}

void ImuSim::run() {
    
    ROS_INFO("imu_sim msg: \nIMU Simulator starting:\n\t - Profile: %s \n\t - Noise type: %s", 
        this->m_imu_profile.c_str(), 
        this->m_noise_type.c_str());

    ros::Rate loop_tool(m_loop_rate);

    while(ros::ok()) {
        this->step();
        ros::spinOnce();
        loop_tool.sleep();
    }
}

ImuSim::ImuSim(): m_node_handle(),
            m_pnode_handle("~"),
            m_imu_state(),
            m_generator(IMU_RAND_SEED),
            m_dynamics_pose_subscriber(m_node_handle, m_dynamics_listo_pose_topic, 5),
            m_dynamics_velocity_subscriber(m_node_handle, m_dynamics_listo_velocity_topic, 5),
            m_dynamics_acceleration_subscriber(m_node_handle, m_dynamics_listo_acceleration_topic, 5),
            m_state_subscriber(this->m_dynamics_pose_subscriber, 
                                this->m_dynamics_velocity_subscriber,
                                this->m_dynamics_acceleration_subscriber, 10)
{

    this->m_imu_sim_data_publisher = this->m_node_handle.advertise<sensor_msgs::Imu>("imu/data", 1000);
    
    this->m_state_subscriber.registerCallback(boost::bind(&ImuSim::f_extract_dynamics_state, this, _1, _2, _3));

    this->f_load_ros_params();

    if(this->m_noise_type == no_noise) {
        this->f_noise_applicator = &ImuSim::f_add_no_noise;
    } else if(this->m_noise_type == gaussian_noise) {
        this->f_noise_applicator = &ImuSim::f_add_gaussian_noise;
    } else {
        ROS_INFO("imu_sim msg: \nFAILED TO INCLUDE NOISE STRATEGY FOR IMU SIMULATOR");
        ROS_INFO("imu_sim msg: \nIMU SIMULATOR SHUTTING DOWN");
        ROS_INFO("imu_sim msg: \nFAILED WITH:\n\t - Profile: %s \n\t - Noise type: %s,\n\t - Hz: %i",
        this->m_imu_profile.c_str(), 
        this->m_noise_type.c_str(),
        this->m_loop_rate);
        exit(1);
    }
};

ImuSim::~ImuSim() {
    ROS_INFO("IMU_SIM FORMALLY DESTRUCTED");
    //death by free
};