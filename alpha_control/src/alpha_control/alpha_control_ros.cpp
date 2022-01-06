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

    m_system_state = Eigen::VectorXd::Zero(STATE_VECTOR_SIZE);

    m_desired_state = Eigen::VectorXd::Zero(STATE_VECTOR_SIZE);

    // Read tf prefix
    std::string tf_prefix;
    m_pnh.param<std::string>(CONF_TF_PREFIX, tf_prefix, "");
    m_tf_prefix = tf_prefix.empty() ? "" : tf_prefix + "/";

    // Read cg link
    std::string cg_link_id;
    m_pnh.param<std::string>(CONF_CG_LINK,
                             cg_link_id,
                             "cg_link");
    m_cg_link_id = m_tf_prefix + cg_link_id;

    // Read world link
    m_pnh.param<std::string>(CONF_WORLD_LINK, m_world_link_id, "world");

    std::string odometry_topic;
    m_pnh.param<std::string>(CONF_ODOMETRY_SOURCE, odometry_topic, "odometry");

    m_odometry_subscriber = m_nh.subscribe(odometry_topic, 100, &AlphaControlROS::f_odometry_cb, this);

    m_desired_state_subscriber = m_nh.subscribe("control/state/desired", 100, &AlphaControlROS::f_desired_state_cb, this);

    m_current_state_publisher = m_nh.advertise<alpha_control::ControlState>("control/state/current", 100);

    m_error_state_publisher = m_nh.advertise<alpha_control::ControlState>("control/state/error", 100);




    // initialize alpha control object
    m_alpha_control = std::make_shared<AlphaControl>();

}

void AlphaControlROS::f_generate_control_allocation_matrix() {

    // Read generator type
    std::string generator_type;
    m_pnh.param<std::string>(CONF_GENERATOR_TYPE, generator_type,
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

    m_control_allocation_matrix = Eigen::MatrixXd::Zero(STATE_VECTOR_SIZE, (int) m_thrusters.size());

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
    // todo: create a globally accessible list and access it through that instancve.
    std::vector<std::string> axis{"x","y","z","roll","pitch","yaw","u","v","w"};
    std::vector<std::string> gains{"p","i","d", "i_max", "i_min"};

    Eigen::MatrixXd gain_matrix(axis.size(), axis.size());

    for(int i = 0 ; i < axis.size() ; i++) {
        for(int j = 0 ; j < gains.size() ; j++) {
            m_pnh.param<double>(CONF_PID "/" + axis.at(i) + "/" + gains.at(j), gain_matrix(i,j),0);
        }
    }

    m_alpha_control->get_pid()->set_kp(gain_matrix.col(0));
    m_alpha_control->get_pid()->set_ki(gain_matrix.col(1));
    m_alpha_control->get_pid()->set_kd(gain_matrix.col(2));
    m_alpha_control->get_pid()->set_i_max(gain_matrix.col(3));
    m_alpha_control->get_pid()->set_i_min(gain_matrix.col(4));

}

void AlphaControlROS::f_generate_thrusters() {
    // Read all the configuration file to get all the listed thrusters
    std::vector<std::string> thruster_id_list;
    m_pnh.param<decltype(thruster_id_list)>(CONF_THRUSTER_IDS, thruster_id_list, decltype(thruster_id_list)());


    // create thruster objects
    for(const auto& id : thruster_id_list) {
        ThrusterROS::Ptr t = std::make_shared<ThrusterROS>();
        t->set_id(std::string(id));
        m_thrusters.emplace_back(t);
    }

    // initialize thrust command publishers
    for(const auto& t : m_thrusters) {

        // read topic id config for thruster
        std::string thrust_command_topic_id;
        m_pnh.param<std::string>(CONF_THRUST_COMMAND_TOPICS"/" + t->get_id(),
                                 thrust_command_topic_id,
                                 "control/thruster/" + t->get_id() + "/command");
        t->set_thrust_command_topic_id(thrust_command_topic_id);

        // read topic id config for thruster
        std::string thrust_force_topic_id;
        m_pnh.param<std::string>(CONF_THRUSTER_FORCE_TOPICS "/" + t->get_id(),
                                 thrust_force_topic_id,
                                 "control/thruster/" + t->get_id() + "/force");
        t->set_thrust_force_topic_id(thrust_force_topic_id);

        // read polynomials for thruster
        std::vector<double> poly;
        m_pnh.param<std::vector<double>>(CONF_THRUSTER_POLY "/" + t->get_id(), poly, std::vector<double>());
        t->get_poly_solver()->set_coeff(poly);

    }

}

void AlphaControlROS::initialize() {

    m_alpha_control->set_controlled_freedoms(std::vector<int>{
       STATE_U_INDEX,
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

    m_control_rate = std::make_shared<ros::Rate>(10);

    m_alpha_control->set_desired_state(m_desired_state);
    m_alpha_control->set_system_state(m_system_state);

    m_controller_worker = std::thread([this] { f_control_loop(); });

    m_dynconf_pid_server.setCallback(std::bind(&AlphaControlROS::f_dynconf_pid_cb, this, std::placeholders::_1, std::placeholders::_2));

}

void AlphaControlROS::f_generate_control_allocation_from_user() {
    for(const auto& t : m_thrusters) {
        Eigen::VectorXd contribution_vector;
        std::vector<double> v;
        m_pnh.param<decltype(v)>(CONF_CONTROL_ALLOCATION_MATRIX "/" + t->get_id(),
                                 v,
                                 decltype(v)());
        contribution_vector = Eigen::Map<Eigen::VectorXd>(&v[0], (int) v.size());

        t->set_contribution_vector(contribution_vector);
    }
}

void AlphaControlROS::f_generate_control_allocation_from_tf() {


    for(const auto& t : m_thrusters) {
        std::string link_id;
        m_pnh.param<std::string>(CONF_CONTROL_TF "/" + t->get_id(),
                                 link_id,
                                 t->get_id() + "_thruster_link");

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

        Eigen::VectorXd rpyuvw(6);

        rpyuvw.tail(3) = eigen_tf.rotation().col(0);

        // for each axis, roll pitch and yaw
        for(int i = 0 ; i < 3 ; i++ ) {
            int a = (i + 1) % 3;
            int b = (i + 2) % 3;
            float hypotenuse = sqrt(pow(eigen_tf.translation()[a], 2) + powf(eigen_tf.translation()[b], 2 ));
            rpyuvw[i] = hypotenuse * sin(eigen_tf.rotation().eulerAngles(0,1,2)[i]);
        }

        t->set_contribution_vector(rpyuvw);
    }
}

bool AlphaControlROS::f_compute_state() {

    try {
        // Transform center of gravity to world
        auto cg_world = m_transform_buffer.lookupTransform(
                m_world_link_id,
                m_cg_link_id,
                ros::Time::now(),
                ros::Duration(10.0)
        );

        tf2::Quaternion quat;
        quat.setW(cg_world.transform.rotation.w);
        quat.setX(cg_world.transform.rotation.x);
        quat.setY(cg_world.transform.rotation.y);
        quat.setZ(cg_world.transform.rotation.z);

        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        m_system_state(STATE_ROLL_INDEX) = roll;
        m_system_state(STATE_PITCH_INDEX) = pitch;
        m_system_state(STATE_YAW_INDEX) = yaw;

    } catch(tf2::LookupException &e) {
        ROS_WARN_STREAM_THROTTLE(10, e.what());
        return false;
    } catch(tf2::ExtrapolationException& e) {
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

        Eigen::Vector3d uvw;
        uvw(0) = m_odometry_msg.twist.twist.linear.x;
        uvw(1) = m_odometry_msg.twist.twist.linear.y;
        uvw(2) = m_odometry_msg.twist.twist.linear.z;

        uvw = cg_odom_eigen.rotation()  * uvw;

        m_system_state(STATE_U_INDEX) = uvw(0);
        m_system_state(STATE_V_INDEX) = uvw(1);
        m_system_state(STATE_W_INDEX) = uvw(2);

    } catch(tf2::LookupException &e) {
        ROS_WARN_STREAM_THROTTLE(10, e.what());
        return false;
    } catch(tf2::ExtrapolationException& e) {
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

    alpha_control::ControlState e;

    Eigen::VectorXd error_state = m_alpha_control->get_state_error();

    e.roll = error_state(STATE_ROLL_INDEX);
    e.pitch = error_state(STATE_PITCH_INDEX);
    e.yaw = error_state(STATE_YAW_INDEX);
    e.u = error_state(STATE_U_INDEX);
    e.v = error_state(STATE_V_INDEX);
    e.w = error_state(STATE_W_INDEX);

    m_error_state_publisher.publish(e);

    return true;
}


void AlphaControlROS::f_control_loop() {


    double pt = ros::Time::now().toSec();


    while(ros::ok()) {


        m_control_rate->sleep();

        if(not f_compute_state()) {
            continue;
        }

        Eigen::VectorXd needed_forces;

        double dt = ros::Time::now().toSec() - pt;

        if(!m_alpha_control->calculate_needed_forces(needed_forces, dt)) {
            continue;
        }

        for(int i = 0 ; i < m_thrusters.size() ; i++) {
            m_thrusters.at(i)->request_force(needed_forces(i));
        }

        pt = ros::Time::now().toSec();

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

void AlphaControlROS::f_dynconf_pid_cb(alpha_control::PIDConfig &config, uint32_t level) {

    Eigen::VectorXd p(STATE_VECTOR_SIZE);
    Eigen::VectorXd i(STATE_VECTOR_SIZE);
    Eigen::VectorXd d(STATE_VECTOR_SIZE);

    p <<
            config.x_p,
            config.y_p,
            config.z_p,
            config.roll_p,
            config.pitch_p,
            config.yaw_p,
            config.u_p,
            config.v_p,
            config.w_p;


    i <<
            config.x_i,
            config.y_i,
            config.z_i,
            config.roll_i,
            config.pitch_i,
            config.yaw_i,
            config.u_i,
            config.v_i,
            config.w_i;

    d <<
            config.x_d,
            config.y_d,
            config.z_d,
            config.roll_d,
            config.pitch_d,
            config.yaw_d,
            config.u_d,
            config.v_d,
            config.w_d;

    m_alpha_control->get_pid()->set_kp(p);
    m_alpha_control->get_pid()->set_ki(i);
    m_alpha_control->get_pid()->set_kd(d);

}


