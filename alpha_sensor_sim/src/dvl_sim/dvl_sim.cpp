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
*/

#include "dvl_sim.h"

void DvlSim::f_cb_simulation_state(const geometry_msgs::PoseStamped::ConstPtr &pose,
                                   const geometry_msgs::TwistStamped::ConstPtr &vel) {

    vehicle_state.depth = pose->pose.position.z;
    vehicle_state.dist_to_seafloor = m_max_range-vehicle_state.depth;
    
    tf2::Quaternion local_quat;
    double roll, pitch, yaw;
    tf2::fromMsg(pose->pose.orientation, local_quat);
    tf2::Matrix3x3(local_quat).getRPY(roll, pitch, yaw);

    vehicle_state.p = roll;
    vehicle_state.q = pitch;
    vehicle_state.r = yaw;

    vehicle_state.u = vel->twist.linear.x;
    vehicle_state.v = vel->twist.linear.y;
    vehicle_state.w = vel->twist.linear.z;

    vehicle_state.p_dot;
    vehicle_state.q_dot;
    vehicle_state.r_dot;


    alpha_sensor_sim::Transducer msg;

    msg.id = 1000;
    msg.velocity = 1000;
    msg.distance = 1000;
    msg.rssi = 1000;
    msg.nsd = 1000;
    msg.beam_valid = 1000;


    for(const auto& i : m_noise_profiles) {
        switch (i) {
            case NoiseType::Gaussian :
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

    if(m_noise_type == NoiseType::Gaussian) {
        f_apply_noise_density(msg);
    }

    m_dvl_sim_data_publisher.publish(msg);

}

void DvlSim::f_generate_parameters() {

    // get dvl profile
    m_pnh.param<std::string>(DvlSimDict::CONF_PROFILE, m_dvl_profile, "");

    // get dvl link name
    m_pnh.param<std::string>(DvlSimDict::CONF_LINK_NAME, m_link_name, "dvl_link");

    //get node frequency
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
    if(noise_type == DvlSimDict::CONF_NOISE_GAUSSIAN_NOISE) {
        m_noise_type = NoiseType::Gaussian;
    } else if (noise_type == DvlSimDict::CONF_NOISE_NO_NOISE) {
        m_noise_type = NoiseType::None;
    }

    std::vector<std::string> types;
    m_pnh.param<std::vector<std::string>>(DvlSimDict::CONF_NOISE_TYPES, types, std::vector<std::string>());
    for(const auto& i : types) {
        if(i == DvlSimDict::CONF_NOISE_GAUSSIAN_NOISE) {
            m_noise_types.emplace_back(NoiseType::Gaussian);
        } else if (i == DvlSimDict::CONF_NOISE_AXIS_MISALIGNMENT) {
            m_noise_types.emplace_back(NoiseType::AxisMisalignment);
        } else if (i == DvlSimDict::CONF_NOISE_CONSTANT_BIAS) {
            m_noise_types.emplace_back(NoiseType::ConstantBias);
        } else if (i == DvlSimDict::CONF_NOISE_RANDOM_WALK) {
            m_noise_types.emplace_back(NoiseType::RandomWalk);
        }
    }

    //declare temporary local variables
    double linear_velocity_mean;
    double linear_velocity_std;
    double angular_velocity_mean;
    double angular_velocity_std;
    double orientation_mean;
    double orientation_std;

    //get and set Dvl profile parameters

    //convert to expected units in ROS per the description in the dvl.yaml file
    
    //assign distribution types
    
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

    m_generator = std::make_shared<std::mt19937_64>(std::random_device{}());

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

}

void DvlSim::f_apply_constant_bias(alpha_sensor_sim::Transducer &msg) {
    // todo: decide how to compute it
}

void DvlSim::f_apply_axis_misalignment(alpha_sensor_sim::Transducer &msg) {

}

void DvlSim::f_apply_bias_instability(alpha_sensor_sim::Transducer &msg) {
    // todo: under construction
}

void DvlSim::f_apply_random_walk(alpha_sensor_sim::Transducer &msg) {
    // todo: under construction
}