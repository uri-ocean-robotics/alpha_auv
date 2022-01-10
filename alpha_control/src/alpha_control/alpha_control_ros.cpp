#include "alpha_control_ros.h"
#include "exception.hpp"
#include "tf2_eigen/tf2_eigen.h"

#include "cmath"
#include "dictionary.h"



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
    m_pnh.param<std::string>(
            CONF_TF_PREFIX,
            tf_prefix,
            CONF_TF_PREFIX_DEFAULT
    );
    m_tf_prefix = tf_prefix.empty() ? CONF_TF_PREFIX_DEFAULT : tf_prefix + "/";

    // Read cg link
    std::string cg_link_id;
    m_pnh.param<std::string>(
            CONF_CG_LINK,
            cg_link_id,
            CONF_CG_LINK_DEFAULT
    );
    m_cg_link_id = m_tf_prefix + cg_link_id;

    // Read world link
    m_pnh.param<std::string>(
            CONF_WORLD_LINK,
            m_world_link_id,
            CONF_WORLD_LINK_DEFAULT
    );

    std::string odometry_topic;
    m_pnh.param<std::string>(
            CONF_ODOMETRY_SOURCE,
            odometry_topic,
            CONF_ODOMETRY_SOURCE_DEFAULT
    );

    m_odometry_subscriber = m_nh.subscribe(
            odometry_topic,
            100,
            &AlphaControlROS::f_cb_msg_odometry,
            this
    );

    m_desired_state_subscriber = m_nh.subscribe(
            TOPIC_CONTROL_STATE_DESIRED,
            100,
            &AlphaControlROS::f_cb_srv_desired_state,
            this
    );

    m_current_state_publisher = m_nh.advertise<alpha_control::ControlState>(
            TOPIC_CONTROL_STATE_CURRENT,
            100
    );

    m_error_state_publisher = m_nh.advertise<alpha_control::ControlState>(
            TOPIC_CONTROL_STATE_ERROR,
            100
    );

    m_get_control_modes_server = m_nh.advertiseService<alpha_control::GetControlModes::Request, alpha_control::GetControlModes::Response>(
            SERVICE_GET_CONTROL_RULES,
            boost::bind(
                    &AlphaControlROS::f_cb_srv_get_control_modes,
                    this,
                    boost::placeholders::_1,
                    boost::placeholders::_2
            )
    );

    m_set_control_point_server = m_nh.advertiseService<alpha_control::SetControlPoint::Request, alpha_control::SetControlPoint::Response>(
            SERVICE_SET_CONTROL_POINT,
            boost::bind(
                    &AlphaControlROS::f_cb_srv_set_control_point,
                    this,
                    boost::placeholders::_1,
                    boost::placeholders::_2
            )
    );


    m_dynconf_pid_server = boost::make_shared<dynamic_reconfigure::Server<alpha_control::PIDConfig>>(m_config_lock);

    // initialize alpha control object
    m_alpha_control = boost::make_shared<AlphaControl>();

}

void AlphaControlROS::f_generate_control_allocation_matrix() {

    // Read generator type
    std::string generator_type;
    m_pnh.param<std::string>(
            CONF_GENERATOR_TYPE,
            generator_type,
            CONF_GENERATOR_TYPE_OPT_TF
    );

    if(generator_type == CONF_GENERATOR_TYPE_OPT_TF) {
        m_generator_type = GeneratorType::TF;
    } else if (generator_type == CONF_GENERATOR_TYPE_OPT_USER) {
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

    Eigen::MatrixXd gain_matrix(STATE_VECTOR_SIZE, CONF_PID_GAINS_SIZE);

    for(int i = 0 ; STATES[i] != nullptr ; i ++) {
        for(int j = 0 ; CONF_PID_GAINS[j] != nullptr ; j++) {
            std::string p = CONF_PID "/" + std::string(STATES[i]) + "/" + std::string(CONF_PID_GAINS[j]);
            m_pnh.param<double>(p,gain_matrix(i,j),CONF_PID_DEFAULT_ANY);
        }
    }

    m_alpha_control->get_pid()->set_kp(gain_matrix.col(CONF_PID_P_INDEX));
    m_alpha_control->get_pid()->set_ki(gain_matrix.col(CONF_PID_I_INDEX));
    m_alpha_control->get_pid()->set_kd(gain_matrix.col(CONF_PID_D_INDEX));
    m_alpha_control->get_pid()->set_i_max(gain_matrix.col(CONF_PID_I_MAX_INDEX));
    m_alpha_control->get_pid()->set_i_min(gain_matrix.col(CONF_PID_I_MIN_INDEX));

}

void AlphaControlROS::f_generate_thrusters() {
    // Read all the configuration file to get all the listed thrusters
    std::vector<std::string> thruster_id_list;
    m_pnh.param<decltype(thruster_id_list)>(
            CONF_THRUSTER_IDS,
            thruster_id_list,
            decltype(thruster_id_list)()
    );


    // create thruster objects
    for(const auto& id : thruster_id_list) {
        ThrusterROS::Ptr t = boost::make_shared<ThrusterROS>();
        t->set_id(std::string(id));
        m_thrusters.emplace_back(t);
    }

    // initialize thrust command publishers
    for(const auto& t : m_thrusters) {

        // read topic id config for thruster
        std::string thrust_command_topic_id;
        m_pnh.param<std::string>(
                CONF_THRUST_COMMAND_TOPICS"/" + t->get_id(),
                thrust_command_topic_id,
                "control/thruster/" + t->get_id() + "/command");
        t->set_thrust_command_topic_id(thrust_command_topic_id);

        // read topic id config for thruster
        std::string thrust_force_topic_id;
        m_pnh.param<std::string>(
                CONF_THRUSTER_FORCE_TOPICS "/" + t->get_id(),
                thrust_force_topic_id,
                "control/thruster/" + t->get_id() + "/force"
        );
        t->set_thrust_force_topic_id(thrust_force_topic_id);

        // read polynomials for thruster
        std::vector<double> poly;
        m_pnh.param<std::vector<double>>(
                CONF_THRUSTER_POLY "/" + t->get_id(),
                poly,
                std::vector<double>()
        );
        t->get_poly_solver()->set_coeff(poly);

    }

}

void AlphaControlROS::initialize() {

    m_alpha_control->set_controlled_freedoms(std::vector<int>{
        STATE_X_INDEX,
        STATE_Y_INDEX,
        STATE_Z_INDEX
    });

    f_read_control_modes();

    f_generate_thrusters();

    f_generate_control_allocation_matrix();

    f_read_pid_gains();

    std::for_each(m_thrusters.begin(),m_thrusters.end(),
            [](const ThrusterROS::Ptr& t){
                t->initialize();
            }
    );

    m_control_rate = boost::make_shared<ros::Rate>(10);

    m_alpha_control->set_desired_state(m_desired_state);
    m_alpha_control->set_system_state(m_system_state);

    m_controller_worker = boost::thread([this] { f_control_loop(); });

    m_controller_worker.detach();

    f_amend_dynconf();

    m_dynconf_pid_server->setCallback(
            boost::bind(
                    &AlphaControlROS::f_cb_dynconf_pid,
                    this,
                    boost::placeholders::_1,
                    boost::placeholders::_2
            )
    );

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

        auto tf_eigen = tf2::transformToEigen(cg_world);

        // for each thruster compute contribution in earth frame
        for(int i = 0 ; i < m_control_allocation_matrix.cols() ; i++){
            Eigen::Vector3d uvw;
            uvw <<
                    m_control_allocation_matrix(STATE_U_INDEX, i),
                    m_control_allocation_matrix(STATE_V_INDEX, i),
                    m_control_allocation_matrix(STATE_W_INDEX, i);

            Eigen::Vector3d xyz = tf_eigen.rotation() * uvw;

            m_control_allocation_matrix(STATE_X_INDEX, i) = xyz(0);
            m_control_allocation_matrix(STATE_Y_INDEX, i) = xyz(1);
            m_control_allocation_matrix(STATE_Z_INDEX, i) = xyz(2);

        }

        m_system_state(STATE_X_INDEX) = cg_world.transform.translation.x;
        m_system_state(STATE_Y_INDEX) = cg_world.transform.translation.y;
        m_system_state(STATE_Z_INDEX) = cg_world.transform.translation.z;

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
        boost::recursive_mutex::scoped_lock lock(m_odom_lock);

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

    m_alpha_control->update_control_allocation_matrix(m_control_allocation_matrix);


    alpha_control::ControlState s;

    s.control_mode = m_control_mode;
    s.position.x = m_system_state(STATE_X_INDEX);
    s.position.y = m_system_state(STATE_Y_INDEX);
    s.position.z = m_system_state(STATE_Z_INDEX);
    s.orientation.x = m_system_state(STATE_ROLL_INDEX);
    s.orientation.y = m_system_state(STATE_PITCH_INDEX);
    s.orientation.z = m_system_state(STATE_YAW_INDEX);
    s.velocity.x = m_system_state(STATE_U_INDEX);
    s.velocity.y = m_system_state(STATE_V_INDEX);
    s.velocity.z = m_system_state(STATE_W_INDEX);

    m_alpha_control->set_system_state(m_system_state);

    m_current_state_publisher.publish(s);

    alpha_control::ControlState e;

    Eigen::VectorXd error_state = m_alpha_control->get_state_error();

    e.position.x = error_state(STATE_X_INDEX);
    e.position.y = error_state(STATE_Y_INDEX);
    e.position.z = error_state(STATE_Z_INDEX);
    e.orientation.x = error_state(STATE_ROLL_INDEX);
    e.orientation.y = error_state(STATE_PITCH_INDEX);
    e.orientation.z = error_state(STATE_YAW_INDEX);
    e.velocity.x = error_state(STATE_U_INDEX);
    e.velocity.y = error_state(STATE_V_INDEX);
    e.velocity.z = error_state(STATE_W_INDEX);

    m_error_state_publisher.publish(e);

    return true;
}


void AlphaControlROS::f_control_loop() {


    double pt = ros::Time::now().toSec();

    while(ros::ok()) {

        m_control_rate->sleep();

        // Compute state
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

void AlphaControlROS::f_cb_msg_odometry(const nav_msgs::Odometry::ConstPtr &msg) {
    boost::recursive_mutex::scoped_lock lock(m_odom_lock);
    m_odometry_msg = *msg;
}

void AlphaControlROS::f_cb_srv_desired_state(const alpha_control::ControlState::ConstPtr &msg) {
    f_amend_desired_state(*msg);
}

void AlphaControlROS::f_cb_dynconf_pid(alpha_control::PIDConfig &config, uint32_t level) {

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

void AlphaControlROS::f_amend_dynconf() {

    boost::recursive_mutex::scoped_lock lock(m_config_lock);

    auto pid = m_alpha_control->get_pid();


    alpha_control::PIDConfig conf;

    conf.x_p = pid->get_kp()(STATE_X_INDEX);
    conf.y_p = pid->get_kp()(STATE_Y_INDEX);
    conf.z_p = pid->get_kp()(STATE_Z_INDEX);
    conf.roll_p = pid->get_kp()(STATE_ROLL_INDEX);
    conf.pitch_p = pid->get_kp()(STATE_PITCH_INDEX);
    conf.yaw_p = pid->get_kp()(STATE_YAW_INDEX);
    conf.u_p = pid->get_kp()(STATE_U_INDEX);
    conf.v_p = pid->get_kp()(STATE_V_INDEX);
    conf.w_p = pid->get_kp()(STATE_W_INDEX);

    conf.x_i = pid->get_ki()(STATE_X_INDEX);
    conf.y_i = pid->get_ki()(STATE_Y_INDEX);
    conf.z_i = pid->get_ki()(STATE_Z_INDEX);
    conf.roll_i = pid->get_ki()(STATE_ROLL_INDEX);
    conf.pitch_i = pid->get_ki()(STATE_PITCH_INDEX);
    conf.yaw_i = pid->get_ki()(STATE_YAW_INDEX);
    conf.u_i = pid->get_ki()(STATE_U_INDEX);
    conf.v_i = pid->get_ki()(STATE_V_INDEX);
    conf.w_i = pid->get_ki()(STATE_W_INDEX);

    conf.x_d = pid->get_kd()(STATE_X_INDEX);
    conf.y_d = pid->get_kd()(STATE_Y_INDEX);
    conf.z_d = pid->get_kd()(STATE_Z_INDEX);
    conf.roll_d = pid->get_kd()(STATE_ROLL_INDEX);
    conf.pitch_d = pid->get_kd()(STATE_PITCH_INDEX);
    conf.yaw_d = pid->get_kd()(STATE_YAW_INDEX);
    conf.u_d = pid->get_kd()(STATE_U_INDEX);
    conf.v_d = pid->get_kd()(STATE_V_INDEX);
    conf.w_d = pid->get_kd()(STATE_W_INDEX);


    m_dynconf_pid_server->updateConfig(conf);

}

void AlphaControlROS::f_read_control_modes() {

    // Read all the modes from parameter server
    m_pnh.param<std::vector<std::string>>(
            CONF_CONTROL_MODES,
            m_control_modes,
            decltype(m_control_modes)()
    );

    // throw an exception if no modes provided
    if(m_control_modes.empty()) {
        throw control_ros_exception("no control modes provided. controller is clueless about what to control.");
    }

    // Loop through all the modes and break them down
    for(const auto& mode : m_control_modes) {
        std::vector<std::string> rules;
        m_pnh.param<std::vector<std::string>>(
                CONF_CONTROL_RULES "/" + mode,
                rules,
                decltype(rules)()
        );

        alpha_control::ControlMode m;

        m.name = mode;

        for(const auto& dof : rules) {
            for(int si = 0 ; STATES[si] != nullptr ; si++) {
                if(dof == STATES[si]) {
                    m_control_rules[mode].emplace_back(si);
                    m.dofs.emplace_back(dof);
                }
            }
        }

        m_control_modes_msg.modes.emplace_back(m);
    }



    m_control_mode = m_control_modes.front();

    m_alpha_control->update_freedoms(m_control_rules[m_control_mode]);


}

bool AlphaControlROS::f_cb_srv_get_control_modes(alpha_control::GetControlModes::Request &req,
                                                 alpha_control::GetControlModes::Response &resp) {

    if(!m_control_modes.empty()) {
        resp.modes = m_control_modes_msg.modes;
        return true;
    } else {
        return false;
    }

}

bool AlphaControlROS::f_cb_srv_set_control_point(alpha_control::SetControlPoint::Request req,
                                                 alpha_control::SetControlPoint::Response resp) {
    return f_amend_desired_state(req.setpoint);
}

bool AlphaControlROS::f_amend_control_mode(std::string mode) {
    if(!mode.empty()) {
        if(mode == m_control_mode) {
            // nothing should change. Operation valid
            return true;
        }

        if (m_control_rules.find(mode) == m_control_rules.end()) {
            std::string modes;
            for (const auto &i: m_control_modes) {
                modes += " " + i;
            }
            ROS_WARN_STREAM(
                    "Requested mode [" << mode << "] doesn't exist. Available modes: " << modes);

            // mode doesn't exist. Operation invalid
            return false;
        }

        m_control_mode = mode;
        m_alpha_control->update_freedoms(m_control_rules[m_control_mode]);
        // mode is not empty. mode is in the modes list. operation is valid.
        return true;
    } else {

        // its empty, operation valid.
        return true;
    }
}

bool AlphaControlROS::f_amend_desired_state(const alpha_control::ControlState &state) {

    if(!f_amend_control_mode(state.control_mode)) {
        return false;
    }

    m_desired_state(STATE_X_INDEX) = state.position.x;
    m_desired_state(STATE_Y_INDEX) = state.position.y;
    m_desired_state(STATE_Z_INDEX) = state.position.z;
    m_desired_state(STATE_ROLL_INDEX) = state.orientation.x;
    m_desired_state(STATE_PITCH_INDEX) = state.orientation.y;
    m_desired_state(STATE_YAW_INDEX) = state.orientation.z;
    m_desired_state(STATE_U_INDEX) = state.velocity.x;
    m_desired_state(STATE_V_INDEX) = state.velocity.y;
    m_desired_state(STATE_W_INDEX) = state.velocity.z;

    m_alpha_control->update_desired_state(m_desired_state);

    m_desired_state_msg = state;

    return true;
}