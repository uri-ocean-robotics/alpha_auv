#include "alpha_control_ros.h"
#include "exception.hpp"

#include "cmath"
#include "dictionary.h"

std::mutex g_odom_lock;


AlphaControlROS::AlphaControlROS()
        : m_nh(),
        m_pnh("~"),
        m_transform_listener(m_transform_buffer),
        m_generator_type(AlphaControlROS::GeneratorType::UNKNOWN)
{

    m_system_state = Eigen::VectorXf::Zero(STATE_VECTOR_SIZE);

    m_desired_state = Eigen::VectorXf::Zero(STATE_VECTOR_SIZE);

    // Read tf prefix
    std::string tf_prefix;
    m_pnh.param<std::string>("tf_prefix", tf_prefix, "");
    m_tf_prefix = tf_prefix.empty() ? "" : tf_prefix + "/";

    // Read cg link
    std::string cg_link_id;
    m_pnh.param<std::string>("cg_link",
                             cg_link_id,
                             "cg_link");
    m_cg_link_id = m_tf_prefix + cg_link_id;

    // Read world link
    m_pnh.param<std::string>("world_link", m_world_link_id, "world");

    std::string odometry_topic;
    m_pnh.param<std::string>("odometry_source", odometry_topic, "odometry");

    m_odometry_subscriber = m_nh.subscribe(odometry_topic, 100, &AlphaControlROS::f_odometry_cb, this);

    m_desired_state_subscriber = m_nh.subscribe("state/desired", 100, &AlphaControlROS::f_desired_state_cb, this);

    m_current_state_publisher = m_nh.advertise<alpha_control::ControlState>("state/current", 100);

    // initialize alpha control object
    m_alpha_control = std::make_shared<AlphaControl>();

}

void AlphaControlROS::f_generate_control_allocation_matrix() {

    // Read generator type
    std::string generator_type;
    m_pnh.param<std::string>("generator_type", generator_type,
                             CONTROL_ALLOCATION_MATRIX_GENERATOR_TYPE_TF);

    if(generator_type == CONTROL_ALLOCATION_MATRIX_GENERATOR_TYPE_TF) {
        m_generator_type = GeneratorType::TF;
    } else if (generator_type == CONTROL_ALLOCATION_MATRIX_GENERATOR_TYPE_USER) {
        m_generator_type = GeneratorType::USER;
    } else {
        m_generator_type = GeneratorType::UNKNOWN;
    }

    if(m_generator_type == GeneratorType::USER) {
        f_generate_control_allocation_from_user();
    } else if (m_generator_type == GeneratorType::TF) {
        f_generate_control_allocation_from_tf();
    } else {
        throw control_ros_exception("control allocation generation method unspecified");
    }

    // Conduct some checks to see if everything is ready to be initialized
    if(m_thrusters.empty()) {
        throw control_ros_exception("no thruster specified");
    }

    for(int i = 0 ; i < m_thrusters.size() - 1 ; i++ ) {
        if (m_thrusters[i]->get_contribution_vector().size() != m_thrusters[i + 1]->get_contribution_vector().size()) {
            throw control_ros_exception("contribution vector sizes doesn't match");
        }
    }

    m_control_allocation_matrix = Eigen::MatrixXf::Zero(STATE_VECTOR_SIZE, (int) m_thrusters.size());

    if (m_generator_type == GeneratorType::USER) {
        for (int i = 0; i < m_thrusters.size(); i++) {
            m_control_allocation_matrix.col(i).tail(6) = m_thrusters[i]->get_contribution_vector().transpose();
        }
    } else { // if (m_generator_type == GeneratorType::TF)
        for (int i = 0; i < m_thrusters.size(); i++) {
            m_control_allocation_matrix.col(i).tail(6) = m_thrusters[i]->get_contribution_vector().transpose();
        }
    }

    m_alpha_control->set_control_allocation_matrix(m_control_allocation_matrix);

}

void AlphaControlROS::f_read_pid_gains() {

    // This is just a fancy way of reading ros params
    std::vector<std::string> axis{"x","y","z","roll","pitch","yaw","u","v","w"};
    std::vector<std::string> gains{"p","i","d", "max", "min"};

    Eigen::MatrixXf gain_matrix(axis.size(), gains.size());

    for(int i = 0 ; i < axis.size() ; i++) {
        for(int j = 0 ; j < gains.size() ; j++) {
            m_pnh.param<float>("pid/" + axis.at(i) + "/" + gains.at(j), gain_matrix(i,j),0);
        }
    }

    m_alpha_control->get_pid()->set_kp(gain_matrix.col(0).cast<float>());
    m_alpha_control->get_pid()->set_ki(gain_matrix.col(1).cast<float>());
    m_alpha_control->get_pid()->set_kd(gain_matrix.col(2).cast<float>());
    m_alpha_control->get_pid()->set_max(gain_matrix.col(3).cast<float>());
    m_alpha_control->get_pid()->set_min(gain_matrix.col(4).cast<float>());

}

void AlphaControlROS::f_generate_thrusters() {
    // Read all the configuration file to get all the listed thrusters
    std::vector<std::string> thruster_id_list;
    m_pnh.param<decltype(thruster_id_list)>(CONF_THRUSTER_IDS, thruster_id_list, decltype(thruster_id_list)());


    // create thruster objects
    for(const auto& id : thruster_id_list) {
        ThrusterROS::Ptr t = std::make_shared<ThrusterROS>();
        t->set_thruster_id(std::string(id));
        m_thrusters.emplace_back(t);
    }

    // initialize thrust setpoint publishers
    for(const auto& t : m_thrusters) {

        // read topic id config for thruster
        std::string topic_id;
        m_pnh.param<std::string>(CONF_THRUSTER_TOPICS"/" + t->get_thruster_id(),
                                 topic_id,
                                 "control/thruster/" + t->get_thruster_id() + "/setpoint");
        t->set_topic_id(topic_id);


        // read polynomials for thruster
        std::vector<double> poly;
        m_pnh.param<std::vector<double>>(CONF_THRUSTER_POLY "/" + t->get_thruster_id(), poly, std::vector<double>());
        t->get_poly_solver()->set_coeff(poly);

    }

}

void AlphaControlROS::initialize() {

    m_alpha_control->set_controlled_freedoms(std::vector<int>{
        // STATE_U_INDEX,
        STATE_PITCH_INDEX,
        STATE_YAW_INDEX
    });

    f_generate_thrusters();

    f_generate_control_allocation_matrix();

    f_read_pid_gains();

    std::for_each(m_thrusters.begin(),m_thrusters.end(),
            [](const ThrusterROS::Ptr& t){
                t->initialize();
            }
    );

    m_control_rate = std::make_shared<ros::Rate>(20);

    m_alpha_control->set_desired_state(m_desired_state);
    m_alpha_control->set_system_state(m_system_state);

    m_controller_worker = std::thread([this] { f_control_loop(); });



}

void AlphaControlROS::f_generate_control_allocation_from_user() {
    for(const auto& t : m_thrusters) {
        Eigen::VectorXf contribution_vector;
        std::vector<float> v;
        m_pnh.param<decltype(v)>("control_allocation/" + t->get_thruster_id(),
                                 v,
                                 decltype(v)());
        contribution_vector = Eigen::Map<Eigen::VectorXf>(&v[0], (int) v.size());

        t->set_contribution_vector(contribution_vector);
    }
}

void AlphaControlROS::f_generate_control_allocation_from_tf() {


    for(const auto& t : m_thrusters) {
        std::string link_id;
        m_pnh.param<std::string>("control_tf/thruster_links/" + t->get_thruster_id(),
                                 link_id,
                                 t->get_thruster_id() + "_thruster_link");

        t->set_link_id(m_tf_prefix + link_id);
    }

    // For each thruster look up transformation
    for(const auto& t : m_thrusters) {

        auto tf = m_transform_buffer.lookupTransform(
                m_cg_link_id,
                t->get_link_id(),
                ros::Time::now(),
                ros::Duration(10.0)
        );
        auto eigen_tf = tf2::transformToEigen(tf);

        Eigen::VectorXf rpyuvw(6);

        rpyuvw.tail(3) = eigen_tf.rotation().col(0).cast<float>();

        // for each axis, roll pitch and yaw
        for(int i = 0 ; i < 3 ; i++ ) {
            int a = (i + 1) % 3;
            int b = (i + 2) % 3;
            float hypotenuse = sqrtf(powf(eigen_tf.translation().cast<float>()[a], 2) + powf(eigen_tf.translation().cast<float>()[b], 2 ));
            rpyuvw[i] = hypotenuse * sinf(eigen_tf.rotation().cast<float>().eulerAngles(0,1,2)[i]);
        }

        t->set_contribution_vector(rpyuvw);
    }
}

bool AlphaControlROS::f_compute_state() {

    try {
        // Transform center of gravity to world
        auto cg_world = m_transform_buffer.lookupTransform(
                m_cg_link_id,
                m_world_link_id,
                ros::Time::now(),
                ros::Duration(10.0)
        );

        auto cg_world_eigen = tf2::transformToEigen(cg_world);

        Eigen::MatrixXf euler = cg_world_eigen.rotation().eulerAngles(0,1,2).cast<float>();
        m_system_state(STATE_ROLL_INDEX) = euler(0);
        m_system_state(STATE_PITCH_INDEX) = euler(1);
        m_system_state(STATE_YAW_INDEX) = euler(2);

    } catch(tf2::LookupException &e) {
        ROS_WARN_STREAM_THROTTLE(10, e.what());
        return false;
    }


    // Transform from odom to world
    try{
        const std::lock_guard<std::mutex> lock(g_odom_lock);

        auto cg_odom = m_transform_buffer.lookupTransform(
                m_cg_link_id,
                m_odometry_msg.child_frame_id,
                ros::Time::now(),
                ros::Duration(10)
        );

        auto cg_odom_eigen = tf2::transformToEigen(cg_odom);

        Eigen::Vector3f uvw;
        uvw(0) = static_cast<float>(m_odometry_msg.twist.twist.linear.x);
        uvw(1) = static_cast<float>(m_odometry_msg.twist.twist.linear.y);
        uvw(2) = static_cast<float>(m_odometry_msg.twist.twist.linear.z);

        uvw = cg_odom_eigen.rotation().cast<float>() * uvw;

        m_system_state(STATE_U_INDEX) = (float)uvw(0);
        m_system_state(STATE_V_INDEX) = (float)uvw(1);
        m_system_state(STATE_W_INDEX) = (float)uvw(2);

    }catch(tf2::LookupException &e) {
        ROS_WARN_STREAM_THROTTLE(10, e.what());
        return false;
    }

    alpha_control::ControlState s;

    s.roll = m_system_state(STATE_ROLL_INDEX);
    s.pitch = m_system_state(STATE_PITCH_INDEX);
    s.yaw = m_system_state(STATE_YAW_INDEX);
    s.u = m_system_state(STATE_U_INDEX);
    s.v = m_system_state(STATE_V_INDEX);
    s.w = m_system_state(STATE_W_INDEX);

    m_alpha_control->set_system_state(m_system_state);

    m_current_state_publisher.publish(s);

    return true;
}


void AlphaControlROS::f_control_loop() {


    for( ;ros::ok(); m_control_rate->sleep()) {

        if(not f_compute_state()) {
            continue;
        }

        auto needed_forces = m_alpha_control->calculate_needed_forces(
                static_cast<float>(m_control_rate->cycleTime().toSec())
        );

        for(int i = 0 ; i < m_thrusters.size() ; i++) {
            m_thrusters.at(i)->request_force((float)needed_forces(i));
        }

    }

}

void AlphaControlROS::f_odometry_cb(const nav_msgs::Odometry::ConstPtr &msg) {
    const std::lock_guard<std::mutex> lock(g_odom_lock);
    m_odometry_msg = *msg;
}

void AlphaControlROS::f_desired_state_cb(const alpha_control::ControlState::ConstPtr &msg) {
    m_desired_state_msg = *msg;

    m_desired_state(STATE_ROLL_INDEX) = msg->roll;
    m_desired_state(STATE_PITCH_INDEX) = msg->pitch;
    m_desired_state(STATE_YAW_INDEX) = msg->yaw;
    m_desired_state(STATE_U_INDEX) = msg->u;
    m_desired_state(STATE_V_INDEX) = msg->v;
    m_desired_state(STATE_W_INDEX) = msg->w;

    m_alpha_control->set_desired_state(m_desired_state);

}
