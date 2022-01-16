#include "AlphaDriverRos.h"

AlphaDriverRos::AlphaDriverRos() : m_nh(""), m_pnh("~")
{

    f_generate_thrusters();

    m_raw_nmea_sub = m_nh.subscribe("driver/raw_nmea", 100, &AlphaDriverRos::f_raw_nmea_callback, this);

    m_struct_nmea_sub = m_nh.subscribe("driver/struct_nmea", 100, &AlphaDriverRos::f_struct_nmea_callback, this);

    m_raw_nmea_pub = m_nh.advertise<std_msgs::String>("driver/incoming_raw_nmea", 1000);

    m_struct_nmea_pub = m_nh.advertise<alpha_msgs::NMEA>("driver/incoming_struct_nmea", 1000);

    m_thrust_report_pub = m_nh.advertise<geometry_msgs::Vector3Stamped>("driver/thrust_report", 1000);

    m_pressure_pub = m_nh.advertise<alpha_msgs::Pressure>("driver/pressure", 1000);

    m_power_pub = m_nh.advertise<alpha_msgs::Power>("driver/power", 1000);

    m_pnh.param<std::string>("port", m_port, "/dev/ttyACM0");

    m_pnh.param<int>("baud", m_baud, 115200);

    m_pnh.param<double>("security_timeout", m_security_timeout, 3);

    m_driver->set_serial_callback(std::bind(&AlphaDriverRos::f_driver_serial_callback, this, std::placeholders::_1));

}

void AlphaDriverRos::initialize() {

    m_driver = std::make_shared<AlphaDriver>(m_port, m_baud);

}

void AlphaDriverRos::f_generate_thrusters() {

    std::vector<std::string> keys;
    m_nh.getParamNames(keys);

    auto ns = m_pnh.getNamespace();

    auto param_name = ns + "/" + DriverDict::CONF_THRUSTERS;

    for(const auto &i : keys) {
        if(i.find(param_name) != std::string::npos) {
            auto pp = i.substr(param_name.size() + 1, i.size());
            auto name = pp.substr(0, pp.find('/'));
            auto param = pp.substr(pp.find('/') + 1, pp.size());

            if(param.find(DriverDict::CONF_THRUSTERS_PWM_CHANNEL) != std::string::npos) {
                m_nh.getParam(i, m_thrusters[name].pwm_channel);
            } else if (param == DriverDict::CONF_THRUSTERS_TOPIC) {
                m_nh.getParam(i, m_thrusters[name].topic);
            } else {
                // unknown param
            }
        }
    }

    for(const auto &i : m_thrusters) {

        if(i.second.topic.size() == 0) {
            throw alpha_driver_ros_exception("thruster defined without topic id");
        }

        for(const auto &j : m_thrusters) {
            if(i.first == j.first) {
                continue;
            }
            if(i.second.pwm_channel == j.second.pwm_channel) {
                throw alpha_driver_ros_exception("multiple thrusters with same pwm channel found");
            }
            if(i.second.topic == j.second.topic) {
                throw alpha_driver_ros_exception("multiple thrusters with same topic id found");
            }
        }
    }

    for(const auto& i : m_thrusters) {
        auto sub = m_nh.subscribe<std_msgs::Float64>(
                i.second.topic,
                5,
                std::bind(&AlphaDriverRos::f_thrust_cb, this, std::placeholders::_1, i.second.pwm_channel)
        );
        m_thrust_cmd_subscribers.emplace_back(sub);
    }

}

void AlphaDriverRos::f_raw_nmea_callback(const std_msgs::String::ConstPtr &msg) {
    m_driver->send_raw(msg->data);
}

void AlphaDriverRos::f_struct_nmea_callback(const alpha_msgs::NMEA::ConstPtr &msg) {
    NMEA nmea;

    nmea.construct(msg->command.c_str(), (float*)&msg->values[0], msg->values.size());

    std::string message(nmea.get_raw());

    m_driver->send_raw(message);
}

void AlphaDriverRos::f_driver_serial_callback(std::string incoming) {

    std_msgs::String raw_msg;
    raw_msg.data = incoming;
    m_raw_nmea_pub.publish(raw_msg);

    alpha_msgs::NMEA nmea_msg;
    NMEA data;
    data.parse(incoming.c_str());

    if(not data.get_valid()) {
        return;
    }


    nmea_msg.header.stamp = ros::Time::now();
    nmea_msg.command = std::string(data.get_cmd());
    nmea_msg.values = std::vector<float>(data.get_values(), data.get_values() + data.get_argc());
    m_struct_nmea_pub.publish(nmea_msg);

    if(nmea_msg.command == NMEA_BAROMETER_REPORT) {
        alpha_msgs::Pressure p;
        p.header.stamp = ros::Time::now();
        p.depth = nmea_msg.values[0];
        p.temperature = nmea_msg.values[1];
        p.pressure = nmea_msg.values[2];
        m_pressure_pub.publish(p);
    } else if (nmea_msg.command == NMEA_MULTIMETER_REPORT) {
        alpha_msgs::Power p;
        p.header.stamp = ros::Time::now();
        p.voltage = nmea_msg.values[0];
        p.current = nmea_msg.values[1];
        m_power_pub.publish(p);
    } else if (nmea_msg.command == NMEA_THRUST_PWM_REPORT){
        geometry_msgs::Vector3Stamped p;
        p.header.stamp = ros::Time::now();
        p.vector.x = nmea_msg.values[0];
        p.vector.y = nmea_msg.values[1];
        p.vector.z = nmea_msg.values[2];
        m_thrust_report_pub.publish(p);
    }
}

void AlphaDriverRos::f_thrust_cb(const std_msgs::Float64::ConstPtr &msg, uint8_t channel) {

    m_last_thrust_command_time = ros::Time::now();

}

void AlphaDriverRos::f_command_thrust_loop() {

    ros::Rate r(20);
    while(ros::ok()) {

        auto dt = ros::Time::now() - m_last_thrust_command_time;
        if(dt.toSec() > m_security_timeout) {
            std::for_each(m_thrust_per_channel.begin(), m_thrust_per_channel.end(), [](auto& i){ i = 0; });
        }

        for(int i = 0 ; i < m_thrust_per_channel.size() ; i++) {
            m_driver->cmd_pwm(m_thrust_per_channel[i], i);
        }

        r.sleep();
    }

}
