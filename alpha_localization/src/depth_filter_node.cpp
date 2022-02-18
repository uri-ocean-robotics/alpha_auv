#include "depth_filter_node.h"

DepthFilterNode::DepthFilterNode() {
    m_nh.reset(new ros::NodeHandle(""));
    m_pnh.reset(new ros::NodeHandle("~"));


    double A, B, H, Q, R, P, x;

    m_pnh->param<double>("A", A, 1.0);
    m_pnh->param<double>("B", B, 0.0);
    m_pnh->param<double>("H", H, 1.0);
    m_pnh->param<double>("Q", Q, 1e-4);
    m_pnh->param<double>("R", R, 1e-4);
    m_pnh->param<double>("P", P, 1.0);
    m_pnh->param<double>("x", x, 0.0);

    m_filter.reset(new UnoLqe(x, A, B, H, Q, R, P));

    m_depth_subscriber = m_nh->subscribe("depth", 10, &DepthFilterNode::f_depth_callback, this);
    m_depth_publisher = m_nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("depth_filtered", 10);

}

void DepthFilterNode::f_depth_callback(const seal_msgs::Float64StampedConstPtr &msg) {

    m_filter->predict();
    m_filter->update(msg->data);

    double x, p;
    m_filter->state(&x, &p);

    geometry_msgs::PoseWithCovarianceStamped out;

    out.header = msg->header;
    out.pose.pose.position.z = x;

    out.pose.covariance[6 * 2 + 2] = p;

    m_depth_publisher.publish(out);

}


int main(int argc, char* argv[]) {

    ros::init(argc, argv, "depth_filter");

    DepthFilterNode d;

    ros::spin();

    return 0;
}