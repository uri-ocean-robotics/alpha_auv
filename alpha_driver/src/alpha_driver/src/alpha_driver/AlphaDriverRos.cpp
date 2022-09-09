#include "AlphaDriverRos.h"

AlphaDriverRos::AlphaDriverRos() : m_nh(""), m_pnh("~") {

    f_read_config();

    m_raw_nmea_sub = m_nh.subscribe("driver/raw_nmea", 100, &AlphaDriverRos::f_raw_nmea_callback, this);

    m_raw_nmea_pub = m_nh.advertise<std_msgs::String>("driver/incoming_raw_nmea", 1000);

    m_thrust_report_pub = m_nh.advertise<geometry_msgs::Vector3Stamped>("driver/thrust_report", 1000);

    m_pressure_pub = m_nh.advertise<mvp_msgs::Float64Stamped>("pressure", 1000);
    m_depth_pub = m_nh.advertise<mvp_msgs::Float64Stamped>("depth", 1000);
    m_temperature_pub = m_nh.advertise<mvp_msgs::Float64Stamped>("temperature", 1000);

    m_current_pub = m_nh.advertise<mvp_msgs::Float64Stamped>("power/current", 1000);
    m_voltage_pub = m_nh.advertise<mvp_msgs::Float64Stamped>("power/voltage", 1000);
    m_power_pub = m_nh.advertise<mvp_msgs::Float64Stamped>("power/power", 1000);

    m_serial0_sub = m_nh.subscribe<std_msgs::String>("serial/0/in", 100, std::bind(&AlphaDriverRos::f_serial_callback, this, std::placeholders::_1, 0));
    m_serial1_sub = m_nh.subscribe<std_msgs::String>("serial/1/in", 100, std::bind(&AlphaDriverRos::f_serial_callback, this, std::placeholders::_1, 1));

    m_serial0_pub = m_nh.advertise<std_msgs::String>("serial/0/out", 100);
    m_serial1_pub = m_nh.advertise<std_msgs::String>("serial/1/out", 100);

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

void AlphaDriverRos::f_driver_serial_callback(std::string incoming) {

    std_msgs::String raw_msg;
    raw_msg.data = incoming;
    m_raw_nmea_pub.publish(raw_msg);


    NMEA msg;
    msg.parse(incoming.c_str());

    if(not msg.get_valid()) {
        return;
    }

    if(strcmp(msg.get_cmd(), NMEA_BAROMETER_REPORT) == 0) {
        mvp_msgs::Float64Stamped pressure, depth, temperature;

        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "alpha/pressure";

        pressure.header = header;
        temperature.header = header;
        depth.header = header;

        sscanf(msg.get_data(), "%*[^,],%lf,%lf,%lf",
               &pressure.data, &temperature.data, &depth.data);

        m_depth_pub.publish(depth);
        m_pressure_pub.publish(pressure);
        m_temperature_pub.publish(temperature);

    } else if (strcmp(msg.get_cmd(), NMEA_MULTIMETER_REPORT) == 0) {
        mvp_msgs::Float64Stamped current, voltage, power;

        std_msgs::Header header;
        header.stamp = ros::Time::now();

        sscanf(msg.get_data(), "%*[^,],%lf,%lf,%lf",
               &voltage.data, &current.data, &power.data);

        voltage.header = header;

        current.header = header;

        current.header = header;

        m_voltage_pub.publish(voltage);
        m_current_pub.publish(current);
        m_power_pub.publish(power);
    } else if(strcmp(msg.get_cmd(), NMEA_SERIAL0_CMD) == 0) {
        std::string data(msg.get_data());
        std_msgs::String d;
        d.data = data.substr(data.find(',') + 1, data.length());
        m_serial0_pub.publish(d);
    } else if(strcmp(msg.get_cmd(), NMEA_SERIAL1_CMD) == 0) {
        std::string data(msg.get_data());
        std_msgs::String d;
        d.data = data.substr(data.find(',') + 1, data.length());
        m_serial1_pub.publish(d);
    } else {

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

void AlphaDriverRos::f_serial_callback(const std_msgs::String::ConstPtr &msg,
                                       int channel) {

    NMEA* m = new NMEA();
    m->construct("%s,%s", channel == 0 ? NMEA_SERIAL0_CMD : NMEA_SERIAL1_CMD, msg->data.c_str());
    m_driver->send_raw(m->get_raw());

    delete m;
}