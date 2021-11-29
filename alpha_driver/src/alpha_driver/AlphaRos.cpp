#include "AlphaRos.h"
#include "nmea.h"

AlphaRos::AlphaRos() : nh_(""), pnh_("~")
{
    raw_nmea_sub_ = nh_.subscribe("driver/raw_nmea",100, &AlphaRos::raw_nmea_callback, this);

    struct_nmea_sub_ = nh_.subscribe("driver/struct_nmea", 100, &AlphaRos::struct_nmea_callback, this);

    thrust_cmd_sub_ = nh_.subscribe("controller/cmd_vel", 100, &AlphaRos::thrust_cmd_callback, this);

    raw_nmea_pub_ = nh_.advertise<std_msgs::String>("driver/incoming_raw_nmea", 1000);

    struct_nmea_pub_ = nh_.advertise<alpha_msgs::NMEA>("driver/incoming_struct_nmea", 1000);

    thrust_report_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("driver/thrust_report", 1000);

    pressure_pub_ = nh_.advertise<alpha_msgs::Pressure>("driver/pressure", 1000);

    power_pub_ = nh_.advertise<alpha_msgs::Power>("driver/power", 1000);

    pnh_.param<std::string>("port", port_,"/dev/ttyACM0");
    pnh_.param<int>("baud", baud_, 921600);

    driver_ = boost::make_shared<AlphaDriver>(port_, baud_);

    driver_->setSerialCallback(boost::bind(&AlphaRos::driver_serial_callback, this, boost::placeholders::_1));
}

void AlphaRos::raw_nmea_callback(const std_msgs::String::ConstPtr &msg) {
    driver_->sendRaw(msg->data);
}

void AlphaRos::struct_nmea_callback(const alpha_msgs::NMEA::ConstPtr &msg) {
    NMEA nmea;

    nmea.construct(msg->command.c_str(), (float*)&msg->values[0], msg->values.size());

    std::string message(nmea.get_raw());

    driver_->sendRaw(message);
}

void AlphaRos::thrust_cmd_callback(const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
    driver_->cmdPwm(msg->vector.x, msg->vector.y, msg->vector.z);
}

void AlphaRos::driver_serial_callback(std::string incoming) {

    std_msgs::String raw_msg;
    raw_msg.data = incoming;
    raw_nmea_pub_.publish(raw_msg);

    alpha_msgs::NMEA nmea_msg;
    NMEA data;
    data.parse(incoming.c_str());

    if(not data.get_valid()) {
        return;
    }


    nmea_msg.header.stamp = ros::Time::now();
    nmea_msg.command = std::string(data.get_cmd());
    nmea_msg.values = std::vector<float>(data.get_values(), data.get_values() + data.get_argc());
    struct_nmea_pub_.publish(nmea_msg);

    if(nmea_msg.command == NMEA_BAROMETER_REPORT) {
        alpha_msgs::Pressure p;
        p.header.stamp = ros::Time::now();
        p.depth = nmea_msg.values[0];
        p.temperature = nmea_msg.values[1];
        p.pressure = nmea_msg.values[2];
        pressure_pub_.publish(p);
    } else if (nmea_msg.command == NMEA_MULTIMETER_REPORT) {
        alpha_msgs::Power p;
        p.header.stamp = ros::Time::now();
        p.voltage = nmea_msg.values[0];
        p.current = nmea_msg.values[1];
        power_pub_.publish(p);
    } else if (nmea_msg.command == NMEA_THRUST_PWM_REPORT){
        geometry_msgs::Vector3Stamped p;
        p.header.stamp = ros::Time::now();
        p.vector.x = nmea_msg.values[0];
        p.vector.y = nmea_msg.values[1];
        p.vector.z = nmea_msg.values[2];
        thrust_report_pub_.publish(p);
    }
}
