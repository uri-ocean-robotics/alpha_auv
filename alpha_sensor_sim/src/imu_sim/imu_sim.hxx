/*
    organization:
     - Smart Ocean Systems Lab

    authors:
     - raymond turrisi
*/

#include "sensor_msgs/Imu.h"
#include "ros/ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
namespace mf = message_filters;

#include <random>
#include "ros/console.h"

#define e(num) pow(10.0,num)

struct vehicle_state_t {
    double u = 0;
    double v = 0;
    double w = 0;
    double p = 0;
    double q = 0;
    double r = 0;
    double u_dot = 0;
    double v_dot = 0;
    double w_dot = 0;
    double p_dot = 0;
    double q_dot = 0;
    double r_dot = 0;
    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    double roll_dot = 0;
    double pitch_dot = 0;
    double yaw_dot = 0;
    double power = 0;
};

class imu_sim {
    private:
        struct conversions {
            static constexpr double inches_to_meters = 0.0254;
            static constexpr double degs_to_rads = 0.0174533;
        };

        std::string noise_type = "NO_NOISE"; //used as an index for the array of function pointers (fastest)
        vehicle_state_t vehicle_state;

        //ROS handles
        ////associated topic names
        const std::string imu_pubto_topic_data = "imu/data";
        const std::string dynamics_listo_pose_topic = "dynamics/pose";
        const std::string dynamics_listo_velocity_topic = "dynamics/velocity";
        const std::string dynamics_listo_acceleration_topic = "dynamics/acceleration";

        ros::NodeHandle node_handle;
        ros::NodeHandle p_node_handle;

        ////publishers
        ros::Publisher imu_sim_data_publisher;

        ////time syncronized subscribers
        mf::Subscriber<geometry_msgs::PoseStamped> dynamics_pose_subscriber;
        mf::Subscriber<geometry_msgs::TwistStamped> dynamics_velocity_subscriber;
        mf::Subscriber<geometry_msgs::TwistStamped> dynamics_acceleration_subscriber;
        mf::TimeSynchronizer<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped, geometry_msgs::TwistStamped> state_subscriber;

        ////ROS types and utilities
        ros::Time recent_time;
        ros::Rate rate;
        tf2::Quaternion quat;

        ////subscriber callback
        void extract_dynamics_state(const geometry_msgs::TwistStamped::ConstPtr &pose, 
                                    const geometry_msgs::TwistStamped::ConstPtr &vel, 
                                    const geometry_msgs::TwistStamped::ConstPtr &accel) {
            //extract only relevant information from those topics, keep them in the standard vehicle state struct
            this->recent_time = pose->header.stamp;
            this->vehicle_state.roll = pose->twist.angular.x;
            this->vehicle_state.pitch = pose->twist.angular.y;
            this->vehicle_state.yaw = pose->twist.angular.z;
            this->vehicle_state.roll_dot = vel->twist.angular.x;
            this->vehicle_state.pitch_dot = vel->twist.angular.y;
            this->vehicle_state.yaw_dot = vel->twist.angular.z;
            this->vehicle_state.u_dot = accel->twist.linear.x;
            this->vehicle_state.v_dot = accel->twist.linear.y;
            this->vehicle_state.w_dot = accel->twist.linear.z;
        }

        sensor_msgs::Imu get_msg(ros::Time time_now, vehicle_state_t vehicle_state) {
            return (this->*noise_applicator)(time_now, vehicle_state);
        }

        //Gaussian noise implementation
        std::default_random_engine generator;
        std::normal_distribution<double> xsens_lin_dist;
        std::normal_distribution<double> xsens_angvel_dist;
        std::normal_distribution<double> xsens_orientation_dist;        

        //array of function pointers of noise applicators
        using msg_fn = sensor_msgs::Imu (imu_sim::*)(const ros::Time &time_now, const vehicle_state_t &vehicle_state);

        msg_fn noise_applicator;

        //Individual noise application methods
        sensor_msgs::Imu add_no_noise(const ros::Time &time_now, const vehicle_state_t &vehicle_state) {
            
            sensor_msgs::Imu msg;

            msg.header.stamp = time_now;

            this->quat.setRPY(vehicle_state.roll, vehicle_state.pitch, vehicle_state.yaw);
            msg.orientation.w = this->quat.w();
            msg.orientation.x = this->quat.x();
            msg.orientation.y = this->quat.y();
            msg.orientation.z = this->quat.z();

            msg.angular_velocity.x = vehicle_state.p;
            msg.angular_velocity.y = vehicle_state.q;
            msg.angular_velocity.z = vehicle_state.r;

            msg.linear_acceleration.x = vehicle_state.u_dot;
            msg.linear_acceleration.y = vehicle_state.v_dot;
            msg.linear_acceleration.z = vehicle_state.w_dot;

            //todo: determine how covariance is to be implemented and if its needed here
            for(int i : {0,1,2,3,4,5,6,7,8}) {
                msg.orientation_covariance[i] = -1.0;
                msg.angular_velocity_covariance[i] = -1.0;
                msg.linear_acceleration_covariance[i] = -1.0;
            }
            return msg;
        }

        sensor_msgs::Imu  add_gaussian_noise(const ros::Time &time_now, const vehicle_state_t &vehicle_state) {
            sensor_msgs::Imu msg;

            msg.header.stamp = time_now;

            this->quat.setRPY(vehicle_state.roll+this->xsens_orientation_dist(generator), 
                                vehicle_state.pitch+this->xsens_orientation_dist(generator),
                                vehicle_state.yaw+this->xsens_orientation_dist(generator));

            msg.orientation.w = this->quat.w();
            msg.orientation.x = this->quat.x();
            msg.orientation.y = this->quat.y();
            msg.orientation.z = this->quat.z();

            msg.angular_velocity.x = vehicle_state.p+this->xsens_angvel_dist(generator);
            msg.angular_velocity.y = vehicle_state.q+this->xsens_angvel_dist(generator);;
            msg.angular_velocity.z = vehicle_state.r+this->xsens_angvel_dist(generator);;

            msg.linear_acceleration.x = vehicle_state.u_dot+this->xsens_lin_dist(generator);;
            msg.linear_acceleration.y = vehicle_state.v_dot+this->xsens_lin_dist(generator);
            msg.linear_acceleration.z = vehicle_state.w_dot+this->xsens_lin_dist(generator);

            //todo: determine how covariance is to be implemented and if its needed here
            for(int i : {0,1,2,3,4,5,6,7,8}) {
                msg.orientation_covariance[i] = -1.0;
                msg.angular_velocity_covariance[i] = -1.0;
                msg.linear_acceleration_covariance[i] = -1.0;
            }
            return msg;
        }
        
    public:

        imu_sim(std::string _noise_type): node_handle(), 
                                        p_node_handle("~"),
                                        rate(100),
                                        vehicle_state(),
                                        dynamics_pose_subscriber(node_handle, dynamics_listo_pose_topic, 1),
                                        dynamics_velocity_subscriber(node_handle, dynamics_listo_pose_topic, 1),
                                        dynamics_acceleration_subscriber(node_handle, dynamics_listo_pose_topic, 1),
                                        state_subscriber(dynamics_pose_subscriber, 
                                                            dynamics_velocity_subscriber,
                                                            dynamics_acceleration_subscriber, 3)
        {
            
            this->imu_sim_data_publisher = this->node_handle.advertise<sensor_msgs::Imu>("imu/data", 1000);

            //Noise strategy. Currently only no noise, and gaussian noise are included. 
            if(this->noise_type == "NO_NOISE") {
                ROS_DEBUG("XSENS Simulator adding no noise..");
                this->noise_applicator = &imu_sim::add_no_noise;
            } else if(this->noise_type == "GAUSSIAN_NOISE") {
                ROS_DEBUG("XSENS Simulator adding gaussian noise..");
                //assign noise applicator
                this->noise_applicator = &imu_sim::add_gaussian_noise;
                //associated parameters
                //+TODO: check units
                double xsens_lin_acc_mean = 0;
                double xsens_lin_acc_std = 60*e(-6)*pow(100.0,0.5); //from datasheet: 60 micro-g/sqrt(Hz), saying the sensor will be ran at 40 Hz
                double xsens_angvel_mean = 0;
                double xsens_angvel_std = 7*conversions::degs_to_rads*e(-3)*pow(100.0,0.5); //from datasheet: 0.007 deg/sec/sqrt(Hz)
                double xsens_orientation_mean = 0;
                double xsens_orientation_std = 0.5*conversions::degs_to_rads; //from datasheet: Roll, pitch = 0.2 deg RMS, Yaw/heading = 0.8 deg RMS

                //assign distribution types
                this->xsens_lin_dist =  std::normal_distribution<double>(xsens_lin_acc_mean, xsens_lin_acc_std);
                this->xsens_angvel_dist =  std::normal_distribution<double>(xsens_angvel_mean, xsens_angvel_std);
                this->xsens_orientation_dist =  std::normal_distribution<double>(xsens_orientation_mean, xsens_orientation_std);
            } else {
                ROS_DEBUG("FAILED TO INCLUDE NOISE STRATEGY FOR XSENS SIMULATOR");
                ROS_DEBUG("XSENS_SIM KILLED SIMULATOR");
                exit(1);
            }
        };

        ~imu_sim() {
            //death by free
        };
        
        void step() {
            imu_sim_data_publisher.publish(this->get_msg(this->recent_time, this->vehicle_state));
            rate.sleep();
        }
};