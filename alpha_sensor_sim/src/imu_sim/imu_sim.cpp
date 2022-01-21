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

    Circa:
     - Spring 2022

    Description: 
     - Part of the alpha_sensor_sim package. Listens to simulator
      topics and adds noise to the data which represents ideal conditions. Different 
      profiles are loaded per the AHRS/IMU's datasheet that is used. 
*/

#include "imu_sim.h"

void ImuSim::f_cb_simulation_state(const geometry_msgs::PoseStamped::ConstPtr &pose,
                                   const geometry_msgs::TwistStamped::ConstPtr &vel,
                                   const geometry_msgs::TwistStamped::ConstPtr &accel) {

    sensor_msgs::Imu msg;

    msg.header.stamp = pose->header.stamp;
    msg.header.frame_id = m_tf_prefix.empty() ? m_link_name : m_tf_prefix + "/" + m_link_name;
    msg.orientation = pose->pose.orientation;
    msg.angular_velocity = vel->twist.angular;
    msg.linear_acceleration = accel->twist.linear;

    msg.orientation_covariance = m_orientation_covariance;
    msg.linear_acceleration_covariance = m_linear_acceleration_covariance;
    msg.angular_velocity_covariance = m_angular_velocity_covariance;

    for(const auto& i : m_noise_types) {
        switch (i) {
            case NoiseType::Gaussian :
                f_apply_gaussian_noise(msg);
                break;
            case NoiseType::RandomWalk:
                f_apply_random_walk(msg);
                break;
            case NoiseType::AxisMisalignment:
                f_apply_axis_misalignment(msg);
                break;
            case NoiseType::ConstantBias:
                f_apply_constant_bias(msg);
                break;
            case NoiseType::None:
                break;
        }
    }

    m_imu_sim_data_publisher.publish(msg);

}

void ImuSim::f_generate_parameters() {

    // get imu profile
    m_pnh.param<std::string>(ImuSimDict::CONF_PROFILE, m_imu_profile, "");

    // get imu link name
    m_pnh.param<std::string>(ImuSimDict::CONF_LINK_NAME, m_link_name, "imu_link");

    //get node frequency
    m_pnh.param<double>(ImuSimDict::CONF_FREQUENCY, m_rate, 100);

    m_pnh.param<std::string>(ImuSimDict::CONF_TF_PREFIX, m_tf_prefix, "");

    double misalignment_x;
    m_pnh.param<double>(
            std::string() + ImuSimDict::CONF_NOISE_AXIS_MISALIGNMENT + "/" + ImuSimDict::CONF_X,
            misalignment_x,
            0
    );

    double misalignment_y;
    m_pnh.param<double>(
            std::string() + ImuSimDict::CONF_NOISE_AXIS_MISALIGNMENT + "/" + ImuSimDict::CONF_Y,
            misalignment_y,
            0
    );

    double misalignment_z;
    m_pnh.param<double>(
            std::string() + ImuSimDict::CONF_NOISE_AXIS_MISALIGNMENT + "/" + ImuSimDict::CONF_Z,
            misalignment_z,
            0
    );
    m_axis_misalignment = Eigen::AngleAxisd(misalignment_x, Eigen::Vector3d::UnitX())
                          * Eigen::AngleAxisd(misalignment_y, Eigen::Vector3d::UnitY())
                          * Eigen::AngleAxisd(misalignment_z, Eigen::Vector3d::UnitZ());

    std::vector<std::string> types;
    m_pnh.param<std::vector<std::string>>(ImuSimDict::CONF_NOISE_TYPES, types, std::vector<std::string>());
    for(const auto& i : types) {
        if(i == ImuSimDict::CONF_NOISE_GAUSSIAN_NOISE) {
            m_noise_types.emplace_back(NoiseType::Gaussian);
        } else if (i == ImuSimDict::CONF_NOISE_AXIS_MISALIGNMENT) {
            m_noise_types.emplace_back(NoiseType::AxisMisalignment);
        } else if (i == ImuSimDict::CONF_NOISE_CONSTANT_BIAS) {
            m_noise_types.emplace_back(NoiseType::ConstantBias);
        } else if (i == ImuSimDict::CONF_NOISE_RANDOM_WALK) {
            m_noise_types.emplace_back(NoiseType::RandomWalk);
        }
    }

    //declare temporary local variables
    double linear_acceleration_mean;
    double linear_acceleration_std;
    double angular_velocity_mean;
    double angular_velocity_std;
    double orientation_mean;
    double orientation_std;

    //get and set imu profile parameters
    m_pnh.param<double>(ImuSimDict::CONF_LINEAR_ACCELERATION_MEAN, linear_acceleration_mean, 0);
    m_pnh.param<double>(ImuSimDict::CONF_LINEAR_ACCELERATION_STD, linear_acceleration_std, 0);
    m_pnh.param<double>(ImuSimDict::CONF_ANGULAR_VELOCITY_MEAN, angular_velocity_mean, 0);
    m_pnh.param<double>(ImuSimDict::CONF_ANGULAR_VELOCITY_STD, angular_velocity_std, 0);
    m_pnh.param<double>(ImuSimDict::CONF_ORIENTATION_MEAN, orientation_mean, 0);
    m_pnh.param<double>(ImuSimDict::CONF_ORIENTATION_STD, orientation_std, 0);

    //convert to expected units in ROS per the description in the imu.yaml file
    double sqrt_freq = sqrt(m_rate);

    linear_acceleration_mean *= conversions::grams_to_meters_over_secsec;
    linear_acceleration_std *= conversions::grams_to_meters_over_secsec * conversions::micro * sqrt_freq;
    angular_velocity_mean *= conversions::degs_to_rads;
    angular_velocity_std *= conversions::degs_to_rads * conversions::milli * sqrt_freq;
    orientation_mean *= conversions::degs_to_rads;
    orientation_std *= conversions::degs_to_rads;
    
    //assign distribution types
    m_linear_acceleration_noise =  std::normal_distribution<double>(
            linear_acceleration_mean,
            linear_acceleration_std);
    m_angular_velocity_noise =  std::normal_distribution<double>(
            angular_velocity_mean,
            angular_velocity_std);
    m_orientation_noise =  std::normal_distribution<double>(
            orientation_mean,
            orientation_std);

    f_compute_covariance_matrix(
            std::vector<double>(3, linear_acceleration_std),
            m_linear_acceleration_covariance);

    f_compute_covariance_matrix(
            std::vector<double>(3, angular_velocity_std),
            m_angular_velocity_covariance);

    f_compute_covariance_matrix(
            std::vector<double>(3, orientation_std),
            m_orientation_covariance);

}

void ImuSim::run() const {
    ros::Rate rate(m_rate);
    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}

ImuSim::ImuSim(): m_nh(),
                  m_pnh("~")
{

    f_generate_parameters();

    m_generator = std::make_shared<std::mt19937_64>(std::random_device{}());

    m_pose_subscriber.subscribe(m_nh, m_topic_pose, 1);

    m_velocity_subscriber.subscribe(m_nh, m_topic_velocity, 1);

    m_acceleration_subscriber.subscribe(m_nh, m_topic_acceleration, 1);

    m_state_subscriber = std::make_shared<StateSynchronizer>(
            m_pose_subscriber,
            m_velocity_subscriber,
            m_acceleration_subscriber,
            1
    );

    m_state_subscriber->registerCallback(
            std::bind(
                    &ImuSim::f_cb_simulation_state,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2,
                    std::placeholders::_3
            )
    );

    m_imu_sim_data_publisher = m_nh.advertise<sensor_msgs::Imu>(m_topic_imu, 1000);
}

void ImuSim::f_apply_gaussian_noise(sensor_msgs::Imu &msg) {

    // Orientation
    tf2::Quaternion noise_offset;
    noise_offset.setRPY(
            m_orientation_noise(*m_generator),
            m_orientation_noise(*m_generator),
            m_orientation_noise(*m_generator)
    );

    tf2::Quaternion orientation{
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
    };

    auto noisy_orientation = orientation * noise_offset;
    msg.orientation.x = noisy_orientation.x();
    msg.orientation.y = noisy_orientation.y();
    msg.orientation.z = noisy_orientation.z();
    msg.orientation.w = noisy_orientation.w();

    // Linear Acceleration
    msg.linear_acceleration.x += m_linear_acceleration_noise(*m_generator);
    msg.linear_acceleration.y += m_linear_acceleration_noise(*m_generator);
    msg.linear_acceleration.z += m_linear_acceleration_noise(*m_generator);

    // Angular Velocity
    msg.angular_velocity.x += m_angular_velocity_noise(*m_generator);
    msg.angular_velocity.y += m_angular_velocity_noise(*m_generator);
    msg.angular_velocity.z += m_angular_velocity_noise(*m_generator);

}

void ImuSim::f_apply_constant_bias(sensor_msgs::Imu &msg) {
    // todo: decide how to compute it
}

void ImuSim::f_apply_bias_instability(sensor_msgs::Imu &msg) {
    // todo: under construction
}

void ImuSim::f_apply_random_walk(sensor_msgs::Imu &msg) {
    // todo: under construction
}

void ImuSim::f_apply_acceleration_bias(sensor_msgs::Imu &msg) {
    // todo: under construction
}

void ImuSim::f_apply_axis_misalignment(sensor_msgs::Imu &msg) {

    Eigen::Quaterniond orientation;
    Eigen::Vector3d linear_acceleration;
    Eigen::Vector3d angular_velocity;

    f_msg_to_eigen(msg, orientation, linear_acceleration, angular_velocity);


    auto mo = orientation * m_axis_misalignment;
    msg.orientation.w = mo.w();
    msg.orientation.x = mo.x();
    msg.orientation.y = mo.y();
    msg.orientation.z = mo.z();

    linear_acceleration = m_axis_misalignment.matrix() * linear_acceleration;
    msg.linear_acceleration.x = linear_acceleration.x();
    msg.linear_acceleration.y = linear_acceleration.y();
    msg.linear_acceleration.z = linear_acceleration.z();

    angular_velocity = m_axis_misalignment.matrix() * angular_velocity;

    msg.angular_velocity.x = angular_velocity.x();
    msg.angular_velocity.y = angular_velocity.y();
    msg.angular_velocity.z = angular_velocity.z();

}

void ImuSim::f_msg_to_eigen(const sensor_msgs::Imu& msg,
                            Eigen::Quaterniond &orientation,
                            Eigen::Vector3d &linear_acceleration,
                            Eigen::Vector3d &angular_velocity)
{

    // orientation
    orientation = Eigen::Quaterniond {
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
    };

    linear_acceleration = Eigen::Vector3d{
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
    };

    angular_velocity = Eigen::Vector3d{
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
    };

}

void ImuSim::f_compute_covariance_matrix(const std::vector<double>& stddev, boost::array<double, 9>& covariance_out) {
    covariance_out = {0};
    covariance_out[0] = pow(stddev[0], 2);
    covariance_out[4] = pow(stddev[1], 2);
    covariance_out[8] = pow(stddev[2], 2);
}
