/*
    Program components:
     - dvl_sim.h
     - <dvl_sim.cpp>
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
     - DVL Simulator node. 
       - Translates to the DVL's velocity by considering the linear and angular velocity
         of the vehicle
       - Simulates a flat seafloor so sensible valid and invalid beams can be considered. 
*/

#include "dvl_sim.h"

void DvlSim::f_cb_simulation_state(const geometry_msgs::PoseStamped::ConstPtr &pose,
                                   const geometry_msgs::TwistStamped::ConstPtr &vel) {
    Eigen::Matrix3d rotation;
    // Acquire transform from baselink to dvl
    try {
        m_transform_stamped = m_tfBuffer.lookupTransform("alpha/dvl", "alpha/base_link",ros::Time(0));
        Eigen::Quaterniond transformation_quat(m_transform_stamped.transform.rotation.x,
                                                m_transform_stamped.transform.rotation.y,
                                                m_transform_stamped.transform.rotation.z,
                                                m_transform_stamped.transform.rotation.w);
        m_rotation_wrt_baselink = transformation_quat.toRotationMatrix();
        m_translation_wrt_baselink << m_transform_stamped.transform.translation.x, 
                                                m_transform_stamped.transform.translation.y, 
                                                m_transform_stamped.transform.translation.z;
    } catch(tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return;
    }
    // pose 
    //// m_depth
    m_depth = pose->pose.position.z;
    m_altitude = m_artificial_seafloor_depth-m_depth;
    
    tf2::Quaternion local_quat;
    double roll, pitch, yaw;
    tf2::fromMsg(pose->pose.orientation, local_quat);

    tf2::Matrix3x3(local_quat).getRPY(roll, pitch, yaw);

    //// orientation
    m_dvl_orientation << roll, pitch, yaw;

    //// velocity
    m_dvl_lin_velocity << vel->twist.linear.x,
                                vel->twist.linear.y,
                                vel->twist.linear.z;

    m_dvl_ang_velocity << vel->twist.angular.x,
                                vel->twist.angular.y,
                                vel->twist.angular.z;

    m_dvl_lin_velocity+=m_dvl_ang_velocity.cross(m_translation_wrt_baselink);

    alpha_sensor_sim::Transducer msg;

    m_reported_distance = f_get_dvl_dist();

    m_reported_vel = f_get_dvl_velocity();
    
    msg.id = pose->header.seq;
    msg.velocity = m_reported_vel;
    msg.distance = m_reported_distance;
    msg.rssi = -1;
    msg.nsd = -1;

    for(const auto& i : m_noise_profiles) {
        switch (i) {
            case NoiseType::Uniform :
                f_apply_uniform_noise(msg);
                break;
            case NoiseType::None:
                break;
        }
    }

    msg.beam_valid = (m_reported_distance < m_max_range) && (m_reported_distance > 0);

    m_dvl_sim_data_publisher.publish(msg);
}

double DvlSim::f_get_dvl_dist() {
    return m_altitude/(cos(m_dvl_orientation[0])*cos(m_dvl_orientation[1]));
}

double DvlSim::f_get_dvl_velocity() {
    double velocity_mag = m_dvl_lin_velocity.norm();
    return velocity_mag;
}

void DvlSim::f_generate_parameters() {

    // get dvl profile
    m_pnh.param<std::string>(DvlSimDict::CONF_PROFILE, m_dvl_profile, "");

    // get dvl link name
    m_pnh.param<std::string>(DvlSimDict::CONF_LINK_NAME, m_link_name, "dvl_link");

    // get node frequency
    m_pnh.param<double>(DvlSimDict::CONF_FREQUENCY, m_rate, 100);

    m_pnh.param<std::string>(DvlSimDict::CONF_TF_PREFIX, m_tf_prefix, "");

    double misalignment_x;
    m_pnh.param<double>(
            std::string() + DvlSimDict::CONF_NOISE_AXIS_MISALIGNMENT + "/" + DvlSimDict::CONF_X,
            misalignment_x,
            0
    );

    double misalignment_y;
    m_pnh.param<double>(
            std::string() + DvlSimDict::CONF_NOISE_AXIS_MISALIGNMENT + "/" + DvlSimDict::CONF_Y,
            misalignment_y,
            0
    );

    double misalignment_z;
    m_pnh.param<double>(
            std::string() + DvlSimDict::CONF_NOISE_AXIS_MISALIGNMENT + "/" + DvlSimDict::CONF_Z,
            misalignment_z,
            0
    );
    m_axis_misalignment = Eigen::AngleAxisd(misalignment_x, Eigen::Vector3d::UnitX())
                          * Eigen::AngleAxisd(misalignment_y, Eigen::Vector3d::UnitY())
                          * Eigen::AngleAxisd(misalignment_z, Eigen::Vector3d::UnitZ());

    //get noise type
    std::string noise_type;
    m_pnh.param<std::string>(DvlSimDict::CONF_NOISE_TYPE, noise_type, "");
    if(noise_type == DvlSimDict::CONF_NOISE_UNIFORM_NOISE) {
        m_noise_type = NoiseType::Uniform;
    } else if (noise_type == DvlSimDict::CONF_NOISE_NO_NOISE) {
        m_noise_type = NoiseType::None;
    }

    std::vector<std::string> types;
    m_pnh.param<std::vector<std::string>>(DvlSimDict::CONF_NOISE_TYPES, types, std::vector<std::string>());
    for(const auto& i : types) {
        if(i == DvlSimDict::CONF_NOISE_UNIFORM_NOISE) {
            m_noise_types.emplace_back(NoiseType::Uniform);
        } else if (i == DvlSimDict::CONF_NOISE_AXIS_MISALIGNMENT) {
            m_noise_types.emplace_back(NoiseType::AxisMisalignment);
        } else if (i == DvlSimDict::CONF_NOISE_CONSTANT_BIAS) {
            m_noise_types.emplace_back(NoiseType::ConstantBias);
        } else if (i == DvlSimDict::CONF_NOISE_RANDOM_WALK) {
            m_noise_types.emplace_back(NoiseType::RandomWalk);
        }
    }

    //declare temporary local variables
    double dvl_max_range = 0;
    double seafloor_depth = 0;
    double velocity_avg_err_percent = 0;
    double distance_avg_err_percent = 0;
    
    //get and set Dvl profile parameters
    m_pnh.param<double>(DvlSimDict::CONF_DVL_MAX_RANGE, dvl_max_range, 0);
    m_pnh.param<double>(DvlSimDict::CONF_SEAFLOOR_DEPTH, seafloor_depth, 0);
    m_pnh.param<double>(DvlSimDict::CONF_NOISE_PERR_VELOCITY, velocity_avg_err_percent, 0);
    m_pnh.param<double>(DvlSimDict::CONF_NOISE_PERR_DISTANCE, distance_avg_err_percent, 0);
    
    //convert to expected units in ROS per the description in the dvl.yaml file
    velocity_avg_err_percent *= convert::percent_to_decimal;
    distance_avg_err_percent *= convert::percent_to_decimal;

    m_artificial_seafloor_depth = seafloor_depth;
    m_max_range = dvl_max_range;

    //assign distribution types
    m_dvl_velocity_noise = std::uniform_real_distribution<double>((-1)*velocity_avg_err_percent, velocity_avg_err_percent);
    m_dvl_distance_noise = std::uniform_real_distribution<double>((-1)*distance_avg_err_percent, distance_avg_err_percent);
}

void DvlSim::run() const {
    ros::Rate rate(m_rate);
    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}

DvlSim::DvlSim(): m_nh(),
                  m_pnh("~"),
                  m_tfListener(m_tfBuffer)
{

    f_generate_parameters();

    m_pose_subscriber.subscribe(m_nh, m_topic_pose, 1);

    m_velocity_subscriber.subscribe(m_nh, m_topic_velocity, 1);

    m_state_subscriber = std::make_shared<StateSynchronizer>(
            m_pose_subscriber,
            m_velocity_subscriber,
            1
    );

    m_state_subscriber->registerCallback(
            std::bind(
                    &DvlSim::f_cb_simulation_state,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2
            )
    );

    m_dvl_sim_data_publisher = m_nh.advertise<alpha_sensor_sim::Transducer>(m_topic_dvl, 1000);


}

void DvlSim::f_apply_uniform_noise(alpha_sensor_sim::Transducer &msg) {
    msg.velocity+=msg.velocity*m_dvl_velocity_noise(m_generator);
    msg.distance+=msg.distance*m_dvl_distance_noise(m_generator);
}