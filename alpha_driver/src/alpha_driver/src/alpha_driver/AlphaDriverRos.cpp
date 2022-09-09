#include "AlphaDriverRos.h"

AlphaDriverRos::AlphaDriverRos() : m_nh(""), m_pnh("~") {

    f_read_config();

    m_raw_nmea_sub = m_nh.subscribe("driver/raw_nmea", 100, &AlphaDriverRos::f_raw_nmea_callback, this);

    m_struct_nmea_sub = m_nh.subscribe("driver/struct_nmea", 100, &AlphaDriverRos::f_struct_nmea_callback, this);

    m_raw_nmea_pub = m_nh.advertise<std_msgs::String>("driver/incoming_raw_nmea", 1000);

    m_struct_nmea_pub = m_nh.advertise<mvp_msgs::NMEA>("driver/incoming_struct_nmea", 1000);

    m_thrust_report_pub = m_nh.advertise<geometry_msgs::Vector3Stamped>("driver/thrust_report", 1000);

    m_pressure_pub = m_nh.advertise<mvp_msgs::Float64Stamped>("pressure", 1000);
    m_depth_pub = m_nh.advertise<mvp_msgs::Float64Stamped>("depth", 1000);
    m_temperature_pub = m_nh.advertise<mvp_msgs::Float64Stamped>("temperature", 1000);

    m_current_pub = m_nh.advertise<mvp_msgs::Float64Stamped>("power/current", 1000);
    m_voltage_pub = m_nh.advertise<mvp_msgs::Float64Stamped>("power/voltage", 1000);
    m_power_pub = m_nh.advertise<mvp_msgs::Float64Stamped>("power/power", 1000);

    f_initialize_topics();

}

void AlphaDriverRos::initialize() {

    m_driver = std::make_shared<AlphaDriver>(m_port, m_baud);

    f_initialize_pwm_channels();

    m_driver->set_serial_callback(std::bind(&AlphaDriverRos::f_driver_serial_callback, this, std::placeholders::_1));

    std::thread t(std::bind(&AlphaDriverRos::f_command_thrust_loop, this));
    t.detach();


}

void AlphaDriverRos::f_read_config() {

    m_pnh.param<std::string>("port", m_port, "/dev/ttyACM0");

    m_pnh.param<int>("baud", m_baud, 115200);

    m_pnh.param<double>("security_timeout", m_security_timeout, 3);

    std::vector<std::string> keys;

    m_nh.getParamNames(keys);

    auto ns = m_pnh.getNamespace();

    auto param_name = ns + "/" + DriverDict::CONF_PWM_CONTROL;

    for(const auto &i : keys) {
        if(i.find(param_name) != std::string::npos) {
            auto pp = i.substr(param_name.size() + 1, i.size());
            auto name = pp.substr(0, pp.find('/'));
            auto param = pp.substr(pp.find('/') + 1, pp.size());

            if(param == DriverDict::CONF_PWM_CHANNEL) {
                int channel;
                m_nh.getParam(i, channel);
                m_pwm_control[name].channel = channel;
            } else if (param == DriverDict::CONF_PWM_TOPIC) {
                std::string topic;
                m_nh.getParam(i, topic);
                m_pwm_control[name].topic = topic;
            } else if (param == DriverDict::CONF_PWM_MODE) {
                std::string mode;
                m_nh.getParam(i, mode);
                if(mode == DriverDict::CONF_PWM_MODE_OPT_THRUSTER) {
                    m_pwm_control[name].mode = static_cast<uint8_t>(PwmMode::Thruster);
                } else if (mode == DriverDict::CONF_PWM_MODE_OPT_PURE) {
                    m_pwm_control[name].mode = static_cast<uint8_t>(PwmMode::Pure);
                } else {

                }
            } else {

            }
        }
    }

    for(const auto &i : m_pwm_control) {
        for(const auto &j : m_pwm_control) {
            if(i.first == j.first) {
                continue;
            }

            if(i.second.topic.empty()) {
                throw alpha_driver_ros_exception("empty topic name");
            }

            if(i.second.channel == j.second.channel) {
                throw alpha_driver_ros_exception("multiple thrusters with same pwm channel found");
            }

            if(i.second.topic == j.second.topic) {
                throw alpha_driver_ros_exception("multiple thrusters with same topic id found");
            }
        }
    }

}

void AlphaDriverRos::f_initialize_pwm_channels() {
    for(const auto& i : m_pwm_control) {
        m_driver->init_pwm(i.second.channel, i.second.mode);
    }
}

void AlphaDriverRos::f_initialize_topics() {

    for(const auto& i : m_pwm_control) {
        auto sub = m_nh.subscribe<std_msgs::Float64>(
                i.second.topic,
                5,
                std::bind(
                        &AlphaDriverRos::f_pwm_cb,
                        this,
                        std::placeholders::_1,
                        i.second.channel,
                        i.second.mode
                )
        );
        m_thrust_cmd_subscribers.emplace_back(sub);
    }
}

void AlphaDriverRos::f_raw_nmea_callback(const std_msgs::String::ConstPtr &msg) {
    m_driver->send_raw(msg->data);
}

void AlphaDriverRos::f_struct_nmea_callback(const mvp_msgs::NMEA::ConstPtr &msg) {
    NMEA nmea;

    nmea.construct(msg->command.c_str(), (float*)&msg->values[0], msg->values.size());

    std::string message(nmea.get_raw());

    m_driver->send_raw(message);
}

void AlphaDriverRos::f_driver_serial_callback(std::string incoming) {

    std_msgs::String raw_msg;
    raw_msg.data = incoming;
    m_raw_nmea_pub.publish(raw_msg);

    mvp_msgs::NMEA nmea_msg;
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
        mvp_msgs::Float64Stamped pressure, depth, temperature;

        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "alpha/pressure";

        pressure.header = header;
        pressure.data = nmea_msg.values[0];
        temperature.header = header;
        temperature.data = nmea_msg.values[1];
        depth.header = header;
        depth.data = nmea_msg.values[2];


        m_depth_pub.publish(depth);
        m_pressure_pub.publish(pressure);
        m_temperature_pub.publish(temperature);

    } else if (nmea_msg.command == NMEA_MULTIMETER_REPORT) {
        mvp_msgs::Float64Stamped current, voltage, power;

        std_msgs::Header header;
        header.stamp = ros::Time::now();

        voltage.header = header;
        voltage.data = nmea_msg.values[0];

        current.header = header;
        current.data = nmea_msg.values[1];

        current.header = header;
        power.data = nmea_msg.values[2];

        m_voltage_pub.publish(voltage);
        m_current_pub.publish(current);
        m_power_pub.publish(power);
    } else if (nmea_msg.command == NMEA_PWM_REPORT){
        // todo: to be defined
    }
}

void AlphaDriverRos::f_pwm_cb(const std_msgs::Float64::ConstPtr &msg, uint16_t channel, uint8_t mode) {

    if(mode == PwmMode::Thruster) {
        m_last_thrust_command_time = ros::Time::now();
    }

    m_driver->cmd_pwm(channel, msg->data);
}

void AlphaDriverRos::f_command_thrust_loop() {

    ros::Rate r(20);
    while(ros::ok()) {

        auto dt = ros::Time::now() - m_last_thrust_command_time;

        if(dt.toSec() > m_security_timeout) {
            for(const auto& i : m_pwm_control) {
                if(i.second.mode == PwmMode::Thruster) {
                    m_driver->cmd_pwm(i.second.channel, 0);
                }
            }
        }

        r.sleep();
    }

}
