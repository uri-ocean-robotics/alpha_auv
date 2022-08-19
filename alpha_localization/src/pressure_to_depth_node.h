#ifndef ALPHA_LOCALIZATION_PRESSURE_TO_DEPTH_H
#define ALPHA_LOCALIZATION_PRESSURE_TO_DEPTH_H

#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/FluidPressure.h"
#include "seal_msgs/Float64Stamped.h"

class PressureToDepthNode{

private:

    ros::NodeHandlePtr m_nh;

    ros::NodeHandlePtr m_pnh;

    ros::Publisher m_depth_publisher;

    ros::Subscriber m_pressure_subscriber;

    std::string m_frame_id;

    double m_fluid_density;

    void f_pressure_callback(const sensor_msgs::FluidPressureConstPtr& msg);

public:

    PressureToDepthNode();

};

#endif
