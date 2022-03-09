#include "alpha_control_ros.h"
#include "exception.hpp"
#include "tf2_eigen/tf2_eigen.h"

#include "dictionary.h"
#include "boost/regex.hpp"


AlphaControlROS::AlphaControlROS()
        : m_nh(),
        m_pnh("~"),
        m_transform_listener(m_transform_buffer),
        m_generator_type(AlphaControlROS::GeneratorType::UNKNOWN)
{

    m_system_state = Eigen::VectorXd::Zero(STATE_VECTOR_SIZE);

    m_desired_state = Eigen::VectorXd::Zero(STATE_VECTOR_SIZE);

    /**
     * Read basic configuration. Configuration regarding to thruster allocation
     * will be read later.
     */

    // Read configuration: enabled
    m_pnh.param<bool>(CONF_ENABLED, m_enabled, false);

    // Read configuration: tf prefix
    std::string tf_prefix;
    m_pnh.param<std::string>(CONF_TF_PREFIX, tf_prefix, CONF_TF_PREFIX_DEFAULT);
    m_tf_prefix = tf_prefix.empty() ? CONF_TF_PREFIX_DEFAULT : tf_prefix + "/";

    // Read configuration: center of gravity link
    std::string cg_link_id;
    m_pnh.param<std::string>(CONF_CG_LINK, cg_link_id, CONF_CG_LINK_DEFAULT);
    m_cg_link_id = m_tf_prefix + cg_link_id;

    // Read configuration: world link
    m_pnh.param<std::string>(
            CONF_WORLD_LINK,
            m_world_link_id,
            CONF_WORLD_LINK_DEFAULT
    );

    // Read configuration: odometry topic id
    std::string odometry_topic;
    m_pnh.param<std::string>(
            CONF_ODOMETRY_SOURCE,
            odometry_topic,
            CONF_ODOMETRY_SOURCE_DEFAULT
    );

    /**
     * Initialize Subscribers
     */
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

    /**
     * Initialize publishers
     */
    m_current_state_publisher = m_nh.advertise<alpha_control::ControlState>(
            TOPIC_CONTROL_STATE_CURRENT,
            100
    );

    m_error_state_publisher = m_nh.advertise<alpha_control::ControlState>(
            TOPIC_CONTROL_STATE_ERROR,
            100
    );

    /**
     * Initialize services
     */
    m_get_control_modes_server = m_nh.advertiseService
        <alpha_control::GetControlModes::Request,
        alpha_control::GetControlModes::Response>
    (
        SERVICE_GET_CONTROL_RULES,
        boost::bind(
            &AlphaControlROS::f_cb_srv_get_control_modes,
            this,
            boost::placeholders::_1,
            boost::placeholders::_2
        )
    );

    m_set_control_point_server = m_nh.advertiseService
        <alpha_control::SetControlPoint::Request,
        alpha_control::SetControlPoint::Response>
    (
        SERVICE_SET_CONTROL_POINT,
        boost::bind(
            &AlphaControlROS::f_cb_srv_set_control_point,
            this,
            boost::placeholders::_1,
            boost::placeholders::_2
        )
    );

    m_enable_controller_server = m_nh.advertiseService
        <std_srvs::Empty::Request,
        std_srvs::Empty::Response>
    (
        SERVICE_CONTROL_ENABLE,
        boost::bind(
            &AlphaControlROS::f_cb_srv_enable,
            this,
            boost::placeholders::_1,
            boost::placeholders::_2
        )
    );

    m_disable_controller_server = m_nh.advertiseService
        <std_srvs::Empty::Request,
        std_srvs::Empty::Response>
    (
        SERVICE_CONTROL_DISABLE,
        boost::bind(
            &AlphaControlROS::f_cb_srv_disable,
            this,
            boost::placeholders::_1,
            boost::placeholders::_2
        )
    );

    /**
     * Initialize dynamic reconfigure server
     */
    m_dynconf_pid_server = boost::make_shared
        <dynamic_reconfigure::Server<alpha_control::PIDConfig>>(m_config_lock);

    /**
     * Initialize the actual controller
     */
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
        throw control_ros_exception(
            "control allocation generation method unspecified"
        );
    }

    // Conduct some checks to see if everything is ready to be initialized
    if(m_thrusters.empty()) {
        throw control_ros_exception("no thruster specified");
    }

    for(int i = 0 ; i < m_thrusters.size() - 1 ; i++ ) {
        if (m_thrusters[i]->get_contribution_vector().size() !=
            m_thrusters[i + 1]->get_contribution_vector().size()) {
            throw control_ros_exception(
                "contribution vector sizes doesn't match"
            );
        }
    }

    m_control_allocation_matrix = Eigen::MatrixXd::Zero(
        STATE_VECTOR_SIZE, (int) m_thrusters.size()
    );

    if (m_generator_type == GeneratorType::USER) {
        for (int i = 0; i < m_thrusters.size(); i++) {
            m_control_allocation_matrix.col(i).tail(6) =
                m_thrusters[i]->get_contribution_vector().transpose();
        }
    } else if (m_generator_type == GeneratorType::TF) {
        for (int i = 0; i < m_thrusters.size(); i++) {
            m_control_allocation_matrix.col(i).tail(6) =
                m_thrusters[i]->get_contribution_vector().transpose();
        }
    } else {
        throw control_ros_exception(
            "thruster allocation matrix generation type is not defined! "
            "Available options are 'user' or 'tf'."
        );

    }

    m_alpha_control->set_control_allocation_matrix(m_control_allocation_matrix);

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
        m_pnh.param<std::string>(std::string()
                + CONF_THRUST_COMMAND_TOPICS + "/" + t->get_id(),
                thrust_command_topic_id,
                "control/thruster/" + t->get_id() + "/command");
        t->set_thrust_command_topic_id(thrust_command_topic_id);

        // read topic id config for thruster
        std::string thrust_force_topic_id;
        m_pnh.param<std::string>(std::string()
                + CONF_THRUSTER_FORCE_TOPICS + "/" + t->get_id(),
                thrust_force_topic_id,
                "control/thruster/" + t->get_id() + "/force"
        );
        t->set_thrust_force_topic_id(thrust_force_topic_id);

        // read polynomials for thruster
        std::vector<double> poly;
        m_pnh.param<std::vector<double>>(std::string()
                + CONF_THRUSTER_POLY + "/" + t->get_id(),
                poly,
                std::vector<double>()
        );
        t->get_poly_solver()->set_coeff(poly);

    }

}

void AlphaControlROS::initialize() {

    f_read_control_modes();

    f_generate_thrusters();

    f_generate_control_allocation_matrix();

    std::for_each(m_thrusters.begin(),m_thrusters.end(),
            [](const ThrusterROS::Ptr& t){
                t->initialize();
            }
    );

    m_controller_frequency = 10;

    m_alpha_control->set_desired_state(m_desired_state);
    m_alpha_control->set_system_state(m_system_state);

    m_controller_worker = boost::thread([this] { f_control_loop(); });

    m_controller_worker.detach();

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

        m_pnh.param<decltype(v)>(
            CONF_CONTROL_ALLOCATION_MATRIX "/" + t->get_id(),
            v,
            decltype(v)()
        );

        contribution_vector =
            Eigen::Map<Eigen::VectorXd>(&v[0], (int) v.size());

        t->set_contribution_vector(contribution_vector);
    }
}

void AlphaControlROS::f_generate_control_allocation_from_tf() {

    for(const auto& t : m_thrusters) {
        std::string link_id;
        m_pnh.param<std::string>(
            CONF_CONTROL_TF "/" + t->get_id(),
            link_id,
            t->get_id() + "_thruster_link"
        );

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
            float hypotenuse = sqrt(
                powf(eigen_tf.translation()[a], 2)
                + powf(eigen_tf.translation()[b], 2 )
            );
            rpyuvw[i] = hypotenuse
                * sin(eigen_tf.rotation().eulerAngles(0,1,2)[i]);
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
                    m_control_allocation_matrix(STATE_SURGE_INDEX, i),
                    m_control_allocation_matrix(STATE_SWAY_INDEX, i),
                    m_control_allocation_matrix(STATE_HEAVE_INDEX, i);

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

        m_system_state(STATE_SURGE_INDEX) = uvw(0);
        m_system_state(STATE_SWAY_INDEX) = uvw(1);
        m_system_state(STATE_HEAVE_INDEX) = uvw(2);

    } catch(tf2::LookupException &e) {
        ROS_WARN_STREAM_THROTTLE(10, e.what());
        return false;
    } catch(tf2::ExtrapolationException& e) {
        ROS_WARN_STREAM_THROTTLE(10, e.what());
        return false;
    }

    m_alpha_control->update_control_allocation_matrix(
        m_control_allocation_matrix
    );


    alpha_control::ControlState s;

    s.control_mode = m_control_mode;
    s.position.x = m_system_state(STATE_X_INDEX);
    s.position.y = m_system_state(STATE_Y_INDEX);
    s.position.z = m_system_state(STATE_Z_INDEX);
    s.orientation.x = m_system_state(STATE_ROLL_INDEX);
    s.orientation.y = m_system_state(STATE_PITCH_INDEX);
    s.orientation.z = m_system_state(STATE_YAW_INDEX);
    s.velocity.x = m_system_state(STATE_SURGE_INDEX);
    s.velocity.y = m_system_state(STATE_SWAY_INDEX);
    s.velocity.z = m_system_state(STATE_HEAVE_INDEX);

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
    e.velocity.x = error_state(STATE_SURGE_INDEX);
    e.velocity.y = error_state(STATE_SWAY_INDEX);
    e.velocity.z = error_state(STATE_HEAVE_INDEX);

    m_error_state_publisher.publish(e);

    return true;
}


void AlphaControlROS::f_control_loop() {

    double pt = ros::Time::now().toSec();

    auto r = ros::Rate(m_controller_frequency);

    while(ros::ok()) {

        /**
         * Thread may not be able to sleep properly. This may happen using
         * simulated time.
         */
        if(!r.sleep()) {
            continue;
        }

        /**
         * Check if controller is enabled or not.
         */
        if(!m_enabled) {
            continue;
        }

        /**
         * Compute the state of the system. Continue on failure. This may
         * happen when transform tree is not ready.
         */
        if(not f_compute_state()) {
            continue;
        }


        Eigen::VectorXd needed_forces;

        /**
         * Get time difference to feed PID controller
         */
        double dt = ros::Time::now().toSec() - pt;

        /**
         * Calculate forces to be requested from thrusters. If operation fails,
         * do not send commands to thrusters.
         */
        if(m_alpha_control->calculate_needed_forces(&needed_forces, dt)) {

            for(int i = 0 ; i < m_thrusters.size() ; i++) {
                m_thrusters.at(i)->request_force(needed_forces(i));
            }

        }

        /**
         * Record the time that loop ends. Later, it will feed the PID
         * controller.
         */
        pt = ros::Time::now().toSec();
    }
}

void AlphaControlROS::f_cb_msg_odometry(
        const nav_msgs::Odometry::ConstPtr &msg) {
    boost::recursive_mutex::scoped_lock lock(m_odom_lock);
    m_odometry_msg = *msg;
}

void AlphaControlROS::f_cb_srv_desired_state(
        const alpha_control::ControlState::ConstPtr &msg) {
    f_amend_desired_state(*msg);
}

void AlphaControlROS::f_cb_dynconf_pid(
        alpha_control::PIDConfig &config, uint32_t level) {

    Eigen::VectorXd p(STATE_VECTOR_SIZE);
    Eigen::VectorXd i(STATE_VECTOR_SIZE);
    Eigen::VectorXd d(STATE_VECTOR_SIZE);
    Eigen::VectorXd i_max(STATE_VECTOR_SIZE);
    Eigen::VectorXd i_min(STATE_VECTOR_SIZE);


    p <<
            config.x_p,
            config.y_p,
            config.z_p,
            config.roll_p,
            config.pitch_p,
            config.yaw_p,
            config.surge_p,
            config.sway_p,
            config.heave_p;


    i <<
            config.x_i,
            config.y_i,
            config.z_i,
            config.roll_i,
            config.pitch_i,
            config.yaw_i,
            config.surge_i,
            config.sway_i,
            config.heave_i;

    d <<
            config.x_d,
            config.y_d,
            config.z_d,
            config.roll_d,
            config.pitch_d,
            config.yaw_d,
            config.surge_d,
            config.sway_d,
            config.heave_d;

    i_max <<
            config.x_i_max,
            config.y_i_max,
            config.z_i_max,
            config.roll_i_max,
            config.pitch_i_max,
            config.yaw_i_max,
            config.surge_i_max,
            config.sway_i_max,
            config.heave_i_max;

    i_min <<
            config.x_i_min,
            config.y_i_min,
            config.z_i_min,
            config.roll_i_min,
            config.pitch_i_min,
            config.yaw_i_min,
            config.surge_i_min,
            config.sway_i_min,
            config.heave_i_min;

    m_alpha_control->get_pid()->set_kp(p);
    m_alpha_control->get_pid()->set_ki(i);
    m_alpha_control->get_pid()->set_kd(d);
    m_alpha_control->get_pid()->set_i_max(i_max);
    m_alpha_control->get_pid()->set_i_min(i_min);

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
    conf.surge_p = pid->get_kp()(STATE_SURGE_INDEX);
    conf.sway_p = pid->get_kp()(STATE_SWAY_INDEX);
    conf.heave_p = pid->get_kp()(STATE_HEAVE_INDEX);

    conf.x_i = pid->get_ki()(STATE_X_INDEX);
    conf.y_i = pid->get_ki()(STATE_Y_INDEX);
    conf.z_i = pid->get_ki()(STATE_Z_INDEX);
    conf.roll_i = pid->get_ki()(STATE_ROLL_INDEX);
    conf.pitch_i = pid->get_ki()(STATE_PITCH_INDEX);
    conf.yaw_i = pid->get_ki()(STATE_YAW_INDEX);
    conf.surge_i = pid->get_ki()(STATE_SURGE_INDEX);
    conf.sway_i = pid->get_ki()(STATE_SWAY_INDEX);
    conf.heave_i = pid->get_ki()(STATE_HEAVE_INDEX);

    conf.x_d = pid->get_kd()(STATE_X_INDEX);
    conf.y_d = pid->get_kd()(STATE_Y_INDEX);
    conf.z_d = pid->get_kd()(STATE_Z_INDEX);
    conf.roll_d = pid->get_kd()(STATE_ROLL_INDEX);
    conf.pitch_d = pid->get_kd()(STATE_PITCH_INDEX);
    conf.yaw_d = pid->get_kd()(STATE_YAW_INDEX);
    conf.surge_d = pid->get_kd()(STATE_SURGE_INDEX);
    conf.sway_d = pid->get_kd()(STATE_SWAY_INDEX);
    conf.heave_d = pid->get_kd()(STATE_HEAVE_INDEX);

    conf.x_i_max = pid->get_i_max()(STATE_X_INDEX);
    conf.y_i_max = pid->get_i_max()(STATE_Y_INDEX);
    conf.z_i_max = pid->get_i_max()(STATE_Z_INDEX);
    conf.roll_i_max = pid->get_i_max()(STATE_ROLL_INDEX);
    conf.pitch_i_max = pid->get_i_max()(STATE_PITCH_INDEX);
    conf.yaw_i_max = pid->get_i_max()(STATE_YAW_INDEX);
    conf.surge_i_max = pid->get_i_max()(STATE_SURGE_INDEX);
    conf.sway_i_max = pid->get_i_max()(STATE_SWAY_INDEX);
    conf.heave_i_max = pid->get_i_max()(STATE_HEAVE_INDEX);

    conf.x_i_min = pid->get_i_min()(STATE_X_INDEX);
    conf.y_i_min = pid->get_i_min()(STATE_Y_INDEX);
    conf.z_i_min = pid->get_i_min()(STATE_Z_INDEX);
    conf.roll_i_min = pid->get_i_min()(STATE_ROLL_INDEX);
    conf.pitch_i_min = pid->get_i_min()(STATE_PITCH_INDEX);
    conf.yaw_i_min = pid->get_i_min()(STATE_YAW_INDEX);
    conf.surge_i_min = pid->get_i_min()(STATE_SURGE_INDEX);
    conf.sway_i_min = pid->get_i_min()(STATE_SWAY_INDEX);
    conf.heave_i_min = pid->get_i_min()(STATE_HEAVE_INDEX);


    m_dynconf_pid_server->updateConfig(conf);

}

void AlphaControlROS::f_read_control_modes() {
    std::vector<std::string> params;
    m_pnh.getParamNames(params);

    /**
     * Read all the modes with regex
     */
    std::set<std::string> modes;
    for (const auto &i: params) {
        boost::regex e{
            std::string() + "(?<=" + CONF_CONTROL_MODES + "/)(\\w+)"};
        boost::smatch w;
        if (boost::regex_search(i, w, e)) {
            modes.insert(w[0]);
        }
    }

    if(modes.empty()) {
        /**
         * There is no mode detected by the control mode parser.
         */
         throw control_ros_exception(
             "No control mode configuration have been found."
         );
    }

    /**
     * Read all the degrees of freedoms by a mode
     */
    std::map<std::string, std::set<int>> mode_rules;
    for (const auto &mode: modes) {
        for (const auto &i: params) {
            if (i.find(std::string() + CONF_CONTROL_MODES + "/" + mode) ==
                std::string::npos) {
                continue;
            }
            boost::regex e{std::string() + "(?<=" + mode + "/)(\\w+)"};
            boost::smatch w;
            if (!boost::regex_search(i, w, e)) {
                continue;
            }
            std::string dof = w[0]; // dof name

            auto found =
                std::find_if(STATE_IDX.begin(), STATE_IDX.end(),
                             [dof](const std::pair<const char *, int> &t) -> bool {
                                          return std::strcmp(dof.c_str(),
                                                             t.first) == 0;
                                      }
            );

            if (found != STATE_IDX.end()) {
                mode_rules[mode].insert(found->second);
            } else {
                throw control_ros_exception(
                        "Unknown freedom name passed '" + dof + "'"
                                                                "Possible values are "
                                                                "'x, y, z, roll, pitch, yaw, surge, sway, heave"
                );
            }
        }
    }

    // Loop through all the modes and break them down
    for (const auto &mode: modes) {
        alpha_control::ControlMode m;

        m.name = mode;

        m.dofs = std::vector<int>(mode_rules[mode].begin(), mode_rules[mode].end());

        for (const auto &dof: mode_rules[mode]) {
            std::string param;
            param += std::string() + CONF_CONTROL_MODES + "/" + mode + "/" +
                     STATES[dof] + "/";
            if (dof == STATE_X_INDEX) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_x.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_x.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_x.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX, m.pid_x.k_i_max, 0);
                m_pnh.param<double>(param + CONF_PID_I_MIN, m.pid_x.k_i_min, 0);
            } else if (dof == STATE_Y_INDEX) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_y.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_y.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_y.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX, m.pid_y.k_i_max, 0);
                m_pnh.param<double>(param + CONF_PID_I_MIN, m.pid_y.k_i_min, 0);
            } else if (dof == STATE_Z_INDEX) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_z.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_z.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_z.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX, m.pid_z.k_i_max, 0);
                m_pnh.param<double>(param + CONF_PID_I_MIN, m.pid_z.k_i_min, 0);
            } else if (dof == STATE_ROLL_INDEX) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_roll.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_roll.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_roll.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX, m.pid_roll.k_i_max,
                                    0);
                m_pnh.param<double>(param + CONF_PID_I_MIN, m.pid_roll.k_i_min,
                                    0);
            } else if (dof == STATE_PITCH_INDEX) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_pitch.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_pitch.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_pitch.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX, m.pid_pitch.k_i_max,
                                    0);
                m_pnh.param<double>(param + CONF_PID_I_MIN, m.pid_pitch.k_i_min,
                                    0);
            } else if (dof == STATE_YAW_INDEX) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_yaw.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_yaw.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_yaw.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX, m.pid_yaw.k_i_max,
                                    0);
                m_pnh.param<double>(param + CONF_PID_I_MIN, m.pid_yaw.k_i_min,
                                    0);
            } else if (dof == STATE_SURGE_INDEX) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_surge.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_surge.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_surge.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX, m.pid_surge.k_i_max,
                                    0);
                m_pnh.param<double>(param + CONF_PID_I_MIN, m.pid_surge.k_i_min,
                                    0);
            } else if (dof == STATE_SWAY_INDEX) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_sway.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_sway.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_sway.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX, m.pid_sway.k_i_max,
                                    0);
                m_pnh.param<double>(param + CONF_PID_I_MIN, m.pid_sway.k_i_min,
                                    0);
            } else if (dof == STATE_HEAVE_INDEX) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_heave.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_heave.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_heave.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX, m.pid_heave.k_i_max,
                                    0);
                m_pnh.param<double>(param + CONF_PID_I_MIN, m.pid_heave.k_i_min,
                                    0);
            }
        }

        m_control_modes.modes.emplace_back(m);

    }

    f_amend_control_mode(*modes.begin());
}


bool AlphaControlROS::f_cb_srv_get_control_modes(
    alpha_control::GetControlModes::Request &req,
    alpha_control::GetControlModes::Response &resp) {

    if(!m_control_modes.modes.empty()) {
        resp.modes = m_control_modes.modes;
        return true;
    } else {
        return false;
    }

}

bool AlphaControlROS::f_cb_srv_set_control_point(
        alpha_control::SetControlPoint::Request req,
        alpha_control::SetControlPoint::Response resp) {

    return f_amend_desired_state(req.setpoint);
}

bool AlphaControlROS::f_cb_srv_enable(
        std_srvs::Empty::Request req, std_srvs::Empty::Response res) {

    ROS_INFO("Controller enabled!");
    m_enabled = true;

    return true;
}

bool AlphaControlROS::f_cb_srv_disable(
        std_srvs::Empty::Request req, std_srvs::Empty::Response res) {

    ROS_INFO("Controller disabled!");
    m_enabled = false;

    return true;
}

bool AlphaControlROS::f_amend_control_mode(std::string mode) {
    if(!mode.empty()) {
        if(mode == m_control_mode) {
            // nothing should change. Operation valid
            return true;
        }

        auto found = std::find_if(
                m_control_modes.modes.begin(),
                m_control_modes.modes.end(),
                [mode](const alpha_control::ControlMode& m) -> bool {
                if(m.name == mode) {
                    return true;
                }
                return false;
            }
        );

        if(found == m_control_modes.modes.end()) {
            ROS_WARN_STREAM(
                    "Requested mode [" << mode << "] doesn't exist. "
            );

            // mode doesn't exist. Operation invalid
            return false;
        }

        // setting PID for requested mode
        alpha_control::PIDConfig pid_conf;
        pid_conf.x_p = found->pid_x.kp;
        pid_conf.x_i = found->pid_x.ki;
        pid_conf.x_d = found->pid_x.kd;
        pid_conf.x_i_max = found->pid_x.k_i_max;
        pid_conf.x_i_min = found->pid_x.k_i_min;

        pid_conf.y_p = found->pid_y.kp;
        pid_conf.y_i = found->pid_y.ki;
        pid_conf.y_d = found->pid_y.kd;
        pid_conf.y_i_max = found->pid_y.k_i_max;
        pid_conf.y_i_min = found->pid_y.k_i_min;

        pid_conf.z_p = found->pid_z.kp;
        pid_conf.z_i = found->pid_z.ki;
        pid_conf.z_d = found->pid_z.kd;
        pid_conf.z_i_max = found->pid_z.k_i_max;
        pid_conf.z_i_min = found->pid_z.k_i_min;

        pid_conf.roll_p = found->pid_roll.kp;
        pid_conf.roll_i = found->pid_roll.ki;
        pid_conf.roll_d = found->pid_roll.kd;
        pid_conf.roll_i_max = found->pid_roll.k_i_max;
        pid_conf.roll_i_min = found->pid_roll.k_i_min;

        pid_conf.pitch_p = found->pid_pitch.kp;
        pid_conf.pitch_i = found->pid_pitch.ki;
        pid_conf.pitch_d = found->pid_pitch.kd;
        pid_conf.pitch_i_max = found->pid_pitch.k_i_max;
        pid_conf.pitch_i_min = found->pid_pitch.k_i_min;

        pid_conf.yaw_p = found->pid_yaw.kp;
        pid_conf.yaw_i = found->pid_yaw.ki;
        pid_conf.yaw_d = found->pid_yaw.kd;
        pid_conf.yaw_i_max = found->pid_yaw.k_i_max;
        pid_conf.yaw_i_min = found->pid_yaw.k_i_min;

        pid_conf.surge_p = found->pid_surge.kp;
        pid_conf.surge_i = found->pid_surge.ki;
        pid_conf.surge_d = found->pid_surge.kd;
        pid_conf.surge_i_max = found->pid_surge.k_i_max;
        pid_conf.surge_i_min = found->pid_surge.k_i_min;

        pid_conf.sway_p = found->pid_sway.kp;
        pid_conf.sway_i = found->pid_sway.ki;
        pid_conf.sway_d = found->pid_sway.kd;
        pid_conf.sway_i_max = found->pid_sway.k_i_max;
        pid_conf.sway_i_min = found->pid_sway.k_i_min;

        pid_conf.heave_p = found->pid_heave.kp;
        pid_conf.heave_i = found->pid_heave.ki;
        pid_conf.heave_d = found->pid_heave.kd;
        pid_conf.heave_i_max = found->pid_heave.k_i_max;
        pid_conf.heave_i_min = found->pid_heave.k_i_min;

        f_cb_dynconf_pid(pid_conf, 0);

        f_amend_dynconf();

        m_control_mode = mode;

        m_alpha_control->update_freedoms(found->dofs);

        ROS_INFO_STREAM("Controller mode changed to " << mode);

        // mode is not empty. mode is in the modes list. operation is valid.
        return true;
    } else {

        // its empty, operation valid.
        return true;
    }
}


bool AlphaControlROS::f_amend_desired_state(
        const alpha_control::ControlState &state) {

    if(!f_amend_control_mode(state.control_mode)) {
        return false;
    }

    m_desired_state(alpha_control::ControlMode::DOF_X) = state.position.x;
    m_desired_state(alpha_control::ControlMode::DOF_Y) = state.position.y;
    m_desired_state(alpha_control::ControlMode::DOF_Z) = state.position.z;
    m_desired_state(alpha_control::ControlMode::DOF_ROLL) = state.orientation.x;
    m_desired_state(alpha_control::ControlMode::DOF_PITCH) = state.orientation.y;
    m_desired_state(alpha_control::ControlMode::DOF_YAW) = state.orientation.z;
    m_desired_state(alpha_control::ControlMode::DOF_SURGE) = state.velocity.x;
    m_desired_state(alpha_control::ControlMode::DOF_SWAY) = state.velocity.y;
    m_desired_state(alpha_control::ControlMode::DOF_HEAVE) = state.velocity.z;

    m_alpha_control->update_desired_state(m_desired_state);

    m_desired_state_msg = state;

    return true;
}