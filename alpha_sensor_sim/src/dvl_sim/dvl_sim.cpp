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
     - 


    +TODO: 
        - [] Make the seafloor depth a parameter
        - [] Make the maximum valid range a parameter
        - [] Add a uniform noise generator, include the +-% error similar to waterlink's datasheet
        - [] Perform transformations with the TF Tree
        - [] 
*/

#include "dvl_sim.h"

dvl_state_t::dvl_state_t() {
    depth = 0;
    dist_to_seafloor = 0;
    dvl_distance = 0;
    dvl_vel = 0;
    orientation = Eigen::Vector3d::Zero();
    lin_velocity = Eigen::Vector3d::Zero();
    ang_velocity = Eigen::Vector3d::Zero();
}

void DvlSim::f_cb_simulation_state(const geometry_msgs::PoseStamped::ConstPtr &pose,
                                   const geometry_msgs::TwistStamped::ConstPtr &vel) {
    
    // pose 
    //// depth
    dvl_state.depth = pose->pose.position.z;
    dvl_state.dist_to_seafloor = m_artificial_seafloor_depth-dvl_state.depth;
    
    tf2::Quaternion local_quat;
    double roll, pitch, yaw;
    tf2::fromMsg(pose->pose.orientation, local_quat);
    tf2::Matrix3x3(local_quat).getRPY(roll, pitch, yaw);

    //// orientation
    dvl_state.orientation[0] = roll;
    dvl_state.orientation[1] = pitch;
    dvl_state.orientation[2] = yaw;

    //// velocity
    dvl_state.lin_velocity[0] = vel->twist.linear.x;
    dvl_state.lin_velocity[1] = vel->twist.linear.y;
    dvl_state.lin_velocity[2] = vel->twist.linear.z;

    dvl_state.ang_velocity[0] = vel->twist.angular.x;
    dvl_state.ang_velocity[1] = vel->twist.angular.y;
    dvl_state.ang_velocity[2] = vel->twist.angular.z;

    alpha_sensor_sim::Transducer msg;

    dvl_state.dvl_distance = f_get_dvl_dist();

    dvl_state.dvl_vel = f_get_dvl_velocity();
    
    msg.id = pose->header.seq;
    msg.velocity = dvl_state.dvl_vel;
    msg.distance = dvl_state.dvl_distance;
    msg.rssi = -1;
    msg.nsd = -1;

    for(const auto& i : m_noise_profiles) {
        switch (i) {
            case NoiseType::Uniform :
                f_apply_noise_density(msg);
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

    msg.beam_valid = (dvl_state.dvl_distance < m_max_range) && (dvl_state.dvl_distance > 0);

    m_dvl_sim_data_publisher.publish(msg);
}

double DvlSim::f_get_dvl_dist() {
    return dvl_state.dist_to_seafloor/(cos(dvl_state.orientation[0])*cos(dvl_state.orientation[1])*cos(dvl_state.orientation[2]));
}

double DvlSim::f_get_dvl_velocity() {
    double velocity_mag = dvl_state.lin_velocity.norm();
    double projected_angular_velocity = dvl_state.dvl_distance*dvl_state.ang_velocity.norm();
    double apparent_beam_velocity = velocity_mag+projected_angular_velocity;
    return apparent_beam_velocity;
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
                  m_pnh("~")
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

void DvlSim::f_apply_noise_density(alpha_sensor_sim::Transducer &msg) {
    msg.velocity+=msg.velocity*m_dvl_velocity_noise(m_generator);
    msg.distance+=msg.distance*m_dvl_distance_noise(m_generator);
}

void DvlSim::f_apply_constant_bias(alpha_sensor_sim::Transducer &msg) {
    // todo: decide how to compute it
}

void DvlSim::f_apply_axis_misalignment(alpha_sensor_sim::Transducer &msg) {
    // todo: ~~
}

void DvlSim::f_apply_bias_instability(alpha_sensor_sim::Transducer &msg) {
    // todo: under construction
}

void DvlSim::f_apply_random_walk(alpha_sensor_sim::Transducer &msg) {
    // todo: under construction
}