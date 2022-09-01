#ifndef ALPHA_LOCALIZATION_FILTER_DEPTH_H
#define ALPHA_LOCALIZATION_FILTER_DEPTH_H

#include "ros/ros.h"
#include "uno_lqe/uno_lqe.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "mvp_msgs/Float64Stamped.h"

class DepthFilterNode{

private:

    UnoLqe::Ptr m_filter;

    ros::NodeHandlePtr m_nh;

    ros::NodeHandlePtr m_pnh;

    ros::Publisher m_depth_publisher;

    ros::Subscriber m_depth_subscriber;

    std::string m_frame_id;

    void f_depth_callback(const mvp_msgs::Float64StampedConstPtr& msg);

public:

    DepthFilterNode();

};

#endif
