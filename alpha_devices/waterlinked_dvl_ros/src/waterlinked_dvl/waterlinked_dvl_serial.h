#ifndef WATERLINKED_DVL_SERIAL_H
#define WATERLINKED_DVL_SERIAL_H

#include "ros/ros.h"
#include "waterlinked_dvl/TransducerReportStamped.h"
#include "waterlinked_dvl/PositionReportStamped.h"
#include "waterlinked_dvl/TransducerStamped.h"
#include "memory"
#include "boost/asio.hpp"

class WaterlinkedDvlSerial {

private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_pnh;

    std::string m_frame_id;

    boost::asio::io_service m_io;
    boost::asio::serial_port m_ser_port;

    std::vector<double> m_velocity_covariance;

    std::vector<double> m_position_covariance;
    ros::Publisher m_transducer_report_publisher;
    ros::Publisher m_transducers_publisher;
    ros::Publisher m_position_report_publisher;
    ros::Publisher m_pose_publisher;

    ros::Publisher m_twist_publisher;
    bool f_checksum(const std::string& incoming);

    std::string f_serial_read();

    void f_serial_parse(std::string incoming);

    void f_clean_up();

public:
    WaterlinkedDvlSerial();


};


#endif