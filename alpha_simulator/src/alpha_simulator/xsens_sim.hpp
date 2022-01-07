/*
    organization:
     - Smart Ocean Systems Lab

    authors:
     - raymond turrisi
    
    current executable objective: 
     - create a mock ahrs class which takes the idealized results from the simulator, 
     produces the corresponding messages that would be produced by the ROS node, and 
     inject noise.
     - this first implementation will be injected into the simulator and an existing process,
     and will eventually become its own node for the simulation. 
     - the first method for injecting noise will be to inject gaussian noise, and support 
     the addition of other methods in the future. 

     - this class is not meant to interface with ros directly, it simply takes the state of the
       simulation and returns what the imu would be returning with noise. The communication of 
       this data is currently left to the simulator.

    read: 
     - https://stats.stackexchange.com/questions/415883/proper-way-to-add-noise-into-a-dataset
     - https://stackoverflow.com/questions/32889309/adding-gaussian-noise
*/

#include "sensor_msgs/Imu.h"
#include "state.hxx"
#include "ros/ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <random>
#include "ros/console.h"

class xsens_sim {
    public:
        //used in constructor to set what type of noise strategy we are applying to the simulation
        static const uint16_t no_noise = 0;
        static const uint16_t gaussian_noise = 1;

    private:
        uint16_t noise_type = 0; //used as an index for the array of function pointers (fastest)

        //Gaussian noise implementation
        std::default_random_engine generator;
        std::normal_distribution<double> xsens_lin_dist;
        std::normal_distribution<double> xsens_angvel_dist;
        std::normal_distribution<double> xsens_orientation_dist;

        //ROS helper for converting an eulerian orientation to quaternion
        tf2::Quaternion quat;

        //array of function pointers of noise applicators
        using msg_fn = sensor_msgs::Imu (xsens_sim::*)(const ros::Time &time_now, const vehicle_state_t &vehicle_state);
        msg_fn noise_applicators[10];

        //TODO: Will use historical values for alternative noise strategies. These terms will change.
        double x_history[10];
        double y_history[10];
        double z_history[10];

        double x_dot_history[10];
        double y_dot_history[10];
        double z_dot_history[10];

        uint64_t ticks_since_conception = 0;
        
        //Individual noise application methods
        sensor_msgs::Imu add_no_noise(const ros::Time &time_now, const vehicle_state_t &vehicle_state) {
            
            sensor_msgs::Imu msg;

            msg.header.stamp = ros::Time::now();

            this->quat.setRPY(vehicle_state.roll, vehicle_state.pitch, vehicle_state.yaw);
            msg.orientation.w = this->quat.w();
            msg.orientation.x = this->quat.x();
            msg.orientation.y = this->quat.y();
            msg.orientation.z = this->quat.z();

            msg.angular_velocity.x = vehicle_state.u;
            msg.angular_velocity.y = vehicle_state.v;
            msg.angular_velocity.z = vehicle_state.w;

            msg.linear_acceleration.x = vehicle_state.u_dot;
            msg.linear_acceleration.y = vehicle_state.v_dot;
            msg.linear_acceleration.z = vehicle_state.w_dot;

            for(int i : {0,1,2,3,4,5,6,7,8}) {
                msg.orientation_covariance[i] = -1.0;
                msg.angular_velocity_covariance[i] = -1.0;
                msg.linear_acceleration_covariance[i] = -1.0;
            }
        }

        sensor_msgs::Imu  add_gaussian_noise(const ros::Time &time_now, const vehicle_state_t &vehicle_state) {
            sensor_msgs::Imu msg;

            msg.header.stamp = ros::Time::now();

            this->quat.setRPY(vehicle_state.roll+this->xsens_orientation_dist(generator), 
                                vehicle_state.pitch+this->xsens_orientation_dist(generator),
                                vehicle_state.yaw+this->xsens_orientation_dist(generator));

            msg.orientation.w = this->quat.w();
            msg.orientation.x = this->quat.x();
            msg.orientation.y = this->quat.y();
            msg.orientation.z = this->quat.z();

            msg.angular_velocity.x = vehicle_state.u+this->xsens_angvel_dist(generator);
            msg.angular_velocity.y = vehicle_state.v+this->xsens_angvel_dist(generator);;
            msg.angular_velocity.z = vehicle_state.w+this->xsens_angvel_dist(generator);;

            msg.linear_acceleration.x = vehicle_state.u_dot+this->xsens_lin_dist(generator);;
            msg.linear_acceleration.y = vehicle_state.v_dot+this->xsens_lin_dist(generator);
            msg.linear_acceleration.z = vehicle_state.w_dot+this->xsens_lin_dist(generator);

            for(int i : {0,1,2,3,4,5,6,7,8}) {
                msg.orientation_covariance[i] = -1.0;
                msg.angular_velocity_covariance[i] = -1.0;
                msg.linear_acceleration_covariance[i] = -1.0;
            }
        }

        
    public:
        
        xsens_sim(uint16_t _noise_type) {
            this->noise_type = _noise_type;
            this->noise_applicators[0] = &xsens_sim::add_no_noise;
            this->noise_applicators[1] = &xsens_sim::add_gaussian_noise;

            //Noise strategy. Currently only no noise, and gaussian noise are included. 
            if(this->noise_type == no_noise) {
                ROS_DEBUG("XSENS Simulator adding no noise..");
            } else if(this->noise_type == gaussian_noise) {
                ROS_DEBUG("XSENS Simulator adding gaussian noise..");
                //TODO: Add values from datasheet
                double xsens_lin_acc_mean = 0;
                double xsens_lin_acc_std = 1;
                double xsens_angvel_mean = 0;
                double xsens_angvel_std = 1;
                double xsens_orientation_mean = 0;
                double xsens_orientation_std = 1;
                                
                this->xsens_lin_dist =  std::normal_distribution<double>(xsens_lin_acc_mean, xsens_lin_acc_std);
                this->xsens_angvel_dist =  std::normal_distribution<double>(xsens_angvel_mean, xsens_angvel_std);
                this->xsens_orientation_dist =  std::normal_distribution<double>(xsens_orientation_mean, xsens_orientation_std);
            } else {
                ROS_DEBUG("FAILED TO INCLUDE NOISE STRATEGY FOR XSENS SIMULATOR");
                ROS_DEBUG("XSENS_SIM KILLED SIMULATOR");
                exit(1);
            }
        };

        ~xsens_sim() {
            //death by free
        };


        //Interface to main simulator (currently)
        sensor_msgs::Imu get_msg(ros::Time time_now, vehicle_state_t vehicle_state) {

            sensor_msgs::Imu msg = (this->*noise_applicators[this->noise_type])(time_now, vehicle_state);
            
            ticks_since_conception+=1;

            return msg;
        }
};