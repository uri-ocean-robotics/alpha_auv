#include "waterlinked_dvl_tcp.h"
#include "json/json.h"
#include "algorithm"
#include "string"
#include "waterlinked_dvl/TransducerReportStamped.h"
#include "waterlinked_dvl/PositionReportStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "utils.hpp"
#include "tf2/LinearMath/Quaternion.h"

WaterlinkedDvlTcp::WaterlinkedDvlTcp() :
        m_nh(),
        m_pnh("~"),
        m_socket(m_io_service)
{

    m_pnh.param<std::string>("ip", m_ip, "192.168.194.95"); // This value is set to fallback ip address
    m_pnh.param<int>("port", m_port, 16171);

    m_pnh.param<std::string>("frame_id", m_frame_id, "dvl_link");

    m_pnh.param<std::vector<double>>("velocity_covariance", m_velocity_covariance, std::vector<double>({}));

    m_pnh.param<std::vector<double>>("position_covariance", m_position_covariance, std::vector<double>({}));

    m_twist_publisher = m_nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("dvl/twist", 1000);

    m_pose_publisher = m_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("dvl/pose", 1000);

    m_transducer_report_publisher = m_nh.advertise<waterlinked_dvl::TransducerReportStamped>("dvl/transducer_report", 1000);

    m_position_report_publisher = m_nh.advertise<waterlinked_dvl::PositionReportStamped>("dvl/position_report", 1000);

    boost::asio::ip::tcp::endpoint  endpoint(
            boost::asio::ip::address::from_string(m_ip), m_port);

    m_socket.connect(endpoint);

    boost::asio::socket_base::receive_buffer_size option(81920);
    m_socket.set_option(option);

    m_reading_thread = boost::thread(
        boost::bind(&WaterlinkedDvlTcp::f_readloop, this)
    );
}


void WaterlinkedDvlTcp::f_readloop() {
    boost::asio::streambuf incoming;
    while(ros::ok()) {
        boost::system::error_code error;
        std::size_t n_read = boost::asio::read_until(m_socket, incoming, '\n', error);


        if(error.value() != 0) {
            continue;
        }

        boost::asio::streambuf::const_buffers_type bufs = incoming.data();
        std::string str(boost::asio::buffers_begin(bufs),
                        boost::asio::buffers_begin(bufs) + n_read);

        incoming.consume(n_read);
        boost::thread t{
            boost::bind(&WaterlinkedDvlTcp::f_parse_and_publish, this, str)
        };

    }
}

void WaterlinkedDvlTcp::f_parse_and_publish(std::string incoming) {

    auto now = ros::Time::now();

    Json::Value root;
    Json::Reader reader;

    try {
        reader.parse(incoming, root);

        auto type = root["type"];
        if(type == "velocity") {
            waterlinked_dvl::TransducerReportStamped msg;
            msg.header.stamp = now;
            msg.header.frame_id = m_frame_id;

            msg.report.time = root["time"].asFloat();
            msg.report.altitude = root["altitude"].asFloat();
            msg.report.status = root["status"].asInt();
            msg.report.fom = root["fom"].asFloat();
            msg.report.velocity_valid = root["velocity_valid"].asBool();
            msg.report.vx = root["vx"].asFloat();
            msg.report.vy = root["vy"].asFloat();
            msg.report.vz = root["vz"].asFloat();
            msg.report.format = root["format"].asString();

            auto transducers_json = root["transducers"];
            for (Json::Value::ArrayIndex i = 0; i != transducers_json.size(); i++) {
                waterlinked_dvl::Transducer transducer;

                transducer.distance = transducers_json[i]["distance"].asFloat();
                transducer.id = transducers_json[i]["id"].asInt();
                transducer.beam_valid = transducers_json[i]["beam_valid"].asBool();

                transducer.nsd = transducers_json[i]["nsd"].asFloat();
                transducer.rssi = transducers_json[i]["rssi"].asFloat();
                transducer.velocity = transducers_json[i]["velocity"].asFloat();

                msg.report.transducers.push_back(transducer);
            }

            m_transducer_report_publisher.publish(msg);

            geometry_msgs::TwistWithCovarianceStamped twist_msg;
            twist_msg.header = msg.header;
            twist_msg.twist.twist.linear.x = msg.report.vx;
            twist_msg.twist.twist.linear.y = msg.report.vy;
            twist_msg.twist.twist.linear.z = msg.report.vz;

            if(m_velocity_covariance.size() == twist_msg.twist.covariance.size()) {
                twist_msg.twist.covariance = as_array<twist_msg.twist.covariance.size()>(m_velocity_covariance);
            }
            m_twist_publisher.publish(twist_msg);

        } else if (type == "position_local") {
            waterlinked_dvl::PositionReportStamped msg;
            msg.header.stamp = now;
            msg.header.frame_id = m_frame_id;
            msg.report.x = root["x"].asFloat();
            msg.report.y = root["y"].asFloat();
            msg.report.z = root["z"].asFloat();

            msg.report.status = root["status"].asInt();
            msg.report.format = root["format"].asString();
            msg.report.roll = root["roll"].asFloat();
            msg.report.pitch = root["pitch"].asFloat();
            msg.report.yaw = root["yaw"].asFloat();
            msg.report.time = root["time"].asFloat();
            msg.report.std = root["std"].asFloat();

            m_position_report_publisher.publish(msg);

            tf2::Quaternion quaternion;
            quaternion.setRPY(msg.report.roll, msg.report.pitch, msg.report.yaw);
            quaternion.normalize();

            geometry_msgs::PoseWithCovarianceStamped pose_msg;
            pose_msg.header = msg.header;
            pose_msg.pose.pose.position.x = msg.report.x;
            pose_msg.pose.pose.position.y = msg.report.y;
            pose_msg.pose.pose.position.z = msg.report.z;

            pose_msg.pose.pose.orientation.w = quaternion.w();
            pose_msg.pose.pose.orientation.x = quaternion.x();
            pose_msg.pose.pose.orientation.y = quaternion.y();
            pose_msg.pose.pose.orientation.z = quaternion.z();
            if(m_position_covariance.size() == pose_msg.pose.covariance.size()) {
                pose_msg.pose.covariance = as_array<pose_msg.pose.covariance.size()>(m_position_covariance);
            }
            m_pose_publisher.publish(pose_msg);

        } else {
            ROS_WARN("Incoming message doesn't have a valid type");
        }

    } catch (Json::LogicError &e){
        ROS_WARN_STREAM("Incoming message couldn't be parsed!: " << e.what());
    }
}
