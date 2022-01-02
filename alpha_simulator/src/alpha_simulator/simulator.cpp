#include "simulator.h"
#include "state.hxx"
#include "cmath"
#include "utilities.hxx"
#include "constants.h"
#include "thread"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/FluidPressure.h"
#include "waterlinked_dvl/TransducerReportStamped.h"
#include "diagnostic_msgs/DiagnosticArray.h"

void Simulator::iterate(control_commands_t cmd) {

    g_thruster.XT = pwm2thrust(cmd.thruster_x);
    g_thruster.YT = pwm2thrust(cmd.thruster_y);
    g_thruster.ZT = pwm2thrust(cmd.thruster_z);

    if(g_world_state_ned.point.z() < 0) {
        g_mass.variable_buoyancy = g_mass.buoyancy * (pow(g_world_state_ned.point.z(),2) * 0.01);
    } else {
        g_mass.variable_buoyancy = g_mass.buoyancy;
    }


    g_inertia.Ixx =
            0.5 * g_mass.hull_m * std::pow(g_vehicle_dimensions.r, 2)
            + g_mass.stationary_battery_m *
              (std::pow(g_mass.stationary_battery_y, 2) + std::pow(g_mass.stationary_battery_z, 2));
    //        + g_mass.movable_battery_m * (std::pow(g_mass.movable_battery_y,2) + std::pow(g_mass.movable_battery_z,2));

    g_inertia.Iyy =
            1.0 / 12 * g_mass.hull_m * std::pow(g_vehicle_dimensions.L, 2)
            + g_mass.stationary_battery_m *
              (std::pow(g_mass.stationary_battery_x, 2) + std::pow(g_mass.stationary_battery_z, 2));
    //        + g_mass.movable_battery_m * (std::pow(g_mass.movable_battery_x,2) + std::pow(g_mass.movable_battery_z,2));

    g_inertia.Izz =
            1.0 / 12 * g_mass.hull_m * std::pow(g_vehicle_dimensions.L, 2)
            + g_mass.stationary_battery_m *
              (std::pow(g_mass.stationary_battery_x, 2) + std::pow(g_mass.stationary_battery_y, 2));
    //         + g_mass.movable_battery_m * (std::pow(g_mass.movable_battery_x,2) + std::pow(g_mass.movable_battery_y,2));

    g_inertia.Ixy = -g_mass.stationary_battery_m * (g_mass.stationary_battery_x * g_mass.stationary_battery_y);
    //                 - g_mass.movable_battery_m * (g_mass.movable_battery_x * g_mass.movable_battery_y);


    g_inertia.Ixz = -g_mass.stationary_battery_m * (g_mass.stationary_battery_x * g_mass.stationary_battery_z);
    //                  - g_mass.movable_battery_m * (g_mass.movable_battery_x * g_mass.movable_battery_z);


    g_inertia.Iyz = -g_mass.stationary_battery_m * (g_mass.stationary_battery_y * g_mass.stationary_battery_z);
    //                  - g_mass.movable_battery_m * (g_mass.movable_battery_y * g_mass.movable_battery_z);

    g_inertia.XG = (g_mass.stationary_battery_m * g_mass.stationary_battery_x +
                    g_mass.movable_battery_m * g_mass.movable_battery_x
                    + g_mass.hull_m * g_mass.hull_x) / g_mass.total;
    g_inertia.YG = (g_mass.stationary_battery_m * g_mass.stationary_battery_y +
                    g_mass.movable_battery_m * g_mass.movable_battery_y
                    + g_mass.hull_m * g_mass.hull_y) / g_mass.total;
    /*
    g_inertia.ZG = (g_mass.stationary_battery_m * g_mass.stationary_battery_z +
                    g_mass.movable_battery_m * g_mass.movable_battery_z
                    + g_mass.hull_m * g_mass.hull_z) / g_mass.total;
    */
    g_inertia.ZG = 1 * INCH_TO_METER;

    g_thruster.MT = -g_thruster.XT * g_inertia.ZG + g_thruster.ZT * (20 * INCH_TO_METER - g_inertia.XG);
    g_thruster.NT = g_thruster.YT * (20 * INCH_TO_METER + g_inertia.XG);

    g_vehicle_state_ned.u_dot = 1.0 / (g_mass.weight / GRAVITY + g_hydro.added_mass[0]) *
                                (g_thruster.XT - g_hydro.damping[0] * g_vehicle_state_ned.u * std::fabs(g_vehicle_state_ned.u)
                             - g_mass.weight / GRAVITY * (0 * g_vehicle_state_ned.w - g_vehicle_state_ned.r * g_vehicle_state_ned.v
                                                       + 0 * g_inertia.ZG
                                                       - (0 + 0) * g_inertia.XG)
                             + (-g_mass.weight + g_mass.variable_buoyancy) * std::sin(g_vehicle_state_ned.pitch));


    g_vehicle_state_ned.v_dot = 1.0 / (g_mass.weight / GRAVITY + g_hydro.added_mass[1]) *
                                (g_thruster.YT - g_hydro.damping[1] * g_vehicle_state_ned.v * std::fabs(g_vehicle_state_ned.v)
                             - g_mass.weight / GRAVITY * (g_vehicle_state_ned.r * g_vehicle_state_ned.u
                                                       + g_vehicle_state_ned.r_dot * g_inertia.XG +
                                                       g_vehicle_state_ned.r * g_inertia.ZG * g_vehicle_state_ned.q));


    g_vehicle_state_ned.w_dot = 1.0 / (g_mass.weight / GRAVITY + g_hydro.added_mass[2]) *
                                (g_thruster.ZT - g_hydro.damping[2] * g_vehicle_state_ned.w * std::fabs(g_vehicle_state_ned.w)
                             - g_mass.weight / GRAVITY *
                               (-g_vehicle_state_ned.q * g_vehicle_state_ned.u - g_vehicle_state_ned.q_dot * g_inertia.XG -
                                std::pow(g_vehicle_state_ned.q, 2) * g_inertia.ZG)
                             - (-g_mass.weight + g_mass.variable_buoyancy) * std::cos(g_vehicle_state_ned.pitch));


    g_vehicle_state_ned.q_dot = 1.0 / (g_inertia.Iyy + g_hydro.added_mass[4]) *
                                (g_thruster.MT - g_hydro.damping[4] * g_vehicle_state_ned.q * std::fabs(g_vehicle_state_ned.q)
                                 - g_inertia.Ixz * std::pow(g_vehicle_state_ned.r, 2)
                                 - g_mass.weight / GRAVITY * (g_inertia.ZG *
                                                       (g_vehicle_state_ned.u_dot + g_vehicle_state_ned.q * g_vehicle_state_ned.w -
                                                        g_vehicle_state_ned.r * g_vehicle_state_ned.v)
                                                       - g_inertia.XG * (g_vehicle_state_ned.w_dot -
                                                                         g_vehicle_state_ned.q * g_vehicle_state_ned.u))
                                 - g_inertia.ZG * g_mass.variable_buoyancy * std::sin(g_vehicle_state_ned.pitch) -
                             g_inertia.XG * g_mass.variable_buoyancy * std::cos(g_vehicle_state_ned.pitch));

    g_vehicle_state_ned.r_dot = 1.0 / (g_inertia.Izz + g_hydro.added_mass[5]) *
                                (g_thruster.NT - g_hydro.damping[5] * g_vehicle_state_ned.r * std::fabs(g_vehicle_state_ned.r)
                             + g_inertia.Ixz * g_vehicle_state_ned.q * g_vehicle_state_ned.r
                             - g_mass.weight / GRAVITY * g_inertia.XG *
                               (g_vehicle_state_ned.v_dot + g_vehicle_state_ned.r * g_vehicle_state_ned.u));

    g_vehicle_state_ned.u += m_dt * g_vehicle_state_ned.u_dot;
    g_vehicle_state_ned.v += m_dt * g_vehicle_state_ned.v_dot;
    g_vehicle_state_ned.w += m_dt * g_vehicle_state_ned.w_dot;

    g_vehicle_state_ned.q += m_dt * g_vehicle_state_ned.q_dot;
    g_vehicle_state_ned.r += m_dt * g_vehicle_state_ned.r_dot;

    g_vehicle_state_ned.pitch_dot = g_vehicle_state_ned.q;
    g_vehicle_state_ned.yaw_dot = std::cos(g_vehicle_state_ned.pitch) * g_vehicle_state_ned.r;
    g_vehicle_state_ned.pitch = g_vehicle_state_ned.pitch + m_dt * g_vehicle_state_ned.pitch_dot;
    g_vehicle_state_ned.yaw = g_vehicle_state_ned.yaw + m_dt * g_vehicle_state_ned.yaw_dot;

    // todo: use wrapTo2Pi like function if doesn't work
    // g_vehicle_state_ned.pitch = std::atan2(std::cos(g_vehicle_state_ned.pitch), std::sin(g_vehicle_state_ned.pitch));
    // g_vehicle_state_ned.yaw = std::atan2(std::cos(g_vehicle_state_ned.yaw), std::sin(g_vehicle_state_ned.yaw));

    // g_vehicle_state_ned.pitch = wrapTo2Pi(g_vehicle_state_ned.pitch);
    // g_vehicle_state_ned.yaw = wrapTo2Pi(g_vehicle_state_ned.yaw);

    g_vehicle_state_ned.pitch = remainder(g_vehicle_state_ned.pitch, 2 * M_PI);
    g_vehicle_state_ned.yaw = remainder(g_vehicle_state_ned.yaw, 2 * M_PI);

    Eigen::Matrix3d rbe = rotz(g_vehicle_state_ned.yaw) * roty(g_vehicle_state_ned.pitch);

    g_world_state_ned.point_dot.x() = rbe.row(0) * Eigen::Vector3d{g_vehicle_state_ned.u, g_vehicle_state_ned.v, g_vehicle_state_ned.w};
    g_world_state_ned.point_dot.y() = rbe.row(1) * Eigen::Vector3d{g_vehicle_state_ned.u, g_vehicle_state_ned.v, g_vehicle_state_ned.w};
    g_world_state_ned.point_dot.z() = rbe.row(2) * Eigen::Vector3d{g_vehicle_state_ned.u, g_vehicle_state_ned.v, g_vehicle_state_ned.w};

    g_world_state_ned.point.x() = g_world_state_ned.point.x() + g_world_state_ned.point_dot.x() * m_dt;
    g_world_state_ned.point.y() = g_world_state_ned.point.y() + g_world_state_ned.point_dot.y() * m_dt;
    g_world_state_ned.point.z() = g_world_state_ned.point.z() + g_world_state_ned.point_dot.z() * m_dt;


    g_mass.W_pump = g_mass.W_pump + g_mass.v_pump * m_dt;
    if (g_mass.W_pump > 0.3) {
        g_mass.W_pump = 0.3;
    } else if (g_mass.W_pump < -0.3) {
        g_mass.W_pump = -0.3;
    }

    g_mass.movable_battery_x = g_mass.movable_battery_x + g_mass.movable_battery_vx * m_dt;
    if (g_mass.movable_battery_x > 3.25 * INCH_TO_METER) {
        g_mass.movable_battery_x = 3.25 * INCH_TO_METER;
    } else if (g_mass.movable_battery_x < -6.5 * INCH_TO_METER) {
        g_mass.movable_battery_x = -.6 * INCH_TO_METER;
    }

}

Simulator::Simulator() : m_nh() , m_pnh("~") {

    m_dt = 0.01; //seconds

    m_simulated_depth = 50;

    m_pnh.param<std::string>("tf_prefix", m_tf_prefix, "");

    m_odom_publisher = m_nh.advertise<nav_msgs::Odometry>("dynamics/odometry", 1000);

    m_diagnostic_publisher = m_nh.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1000);

    m_cmd_subscriber = m_nh.subscribe("controller/cmd_vel", 100, &Simulator::cmd_callback, this);

    m_loop_thread = std::thread(std::bind(&Simulator::loop, this));

    m_100hz_thread = std::thread([this](){
        while(ros::ok()) {
            auto now = std::chrono::system_clock::now();

            publish_odometry();

            std::this_thread::sleep_until(now + std::chrono::milliseconds(10));
        }
    });
    m_100hz_thread.detach();

    m_10hz_thread = std::thread([this](){
        while(ros::ok()) {
            auto now = std::chrono::system_clock::now();

            publish_diagnostics();

            std::this_thread::sleep_until(now + std::chrono::milliseconds(100));
        }
    });
    m_10hz_thread.detach();
}

void Simulator::cmd_callback(const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
    g_controls.thruster_x = (msg->vector.x * 500.0) + 1500.0;
    g_controls.thruster_y = (msg->vector.y * 500.0) + 1500.0;
    g_controls.thruster_z = (msg->vector.z * 500.0) + 1500.0;
}


void Simulator::publish_odometry() {

    nav_msgs::Odometry msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = m_tf_prefix.empty() ? "odom" : m_tf_prefix + "/odom";
    msg.child_frame_id = m_tf_prefix.empty() ? "base_link" : m_tf_prefix + "/base_link";

    auto world_enu = ned_to_enu(g_world_state_ned);

    msg.pose.pose.position.x = g_world_state_ned.point.x();
    msg.pose.pose.position.y = g_world_state_ned.point.y();
    msg.pose.pose.position.z = g_world_state_ned.point.z();

    auto vehicle_enu = ned_to_enu(g_vehicle_state_ned);

    tf2::Quaternion quat;
    quat.setRPY(g_vehicle_state_ned.roll, g_vehicle_state_ned.pitch, g_vehicle_state_ned.yaw);
    msg.pose.pose.orientation.w = quat.w();
    msg.pose.pose.orientation.x = quat.x();
    msg.pose.pose.orientation.y = quat.y();
    msg.pose.pose.orientation.z = quat.z();

    msg.twist.twist.linear.x = g_vehicle_state_ned.u;
    msg.twist.twist.linear.y = g_vehicle_state_ned.v;
    msg.twist.twist.linear.z = g_vehicle_state_ned.w;

    msg.twist.twist.angular.y = g_vehicle_state_ned.q;
    msg.twist.twist.angular.z = g_vehicle_state_ned.r;

    m_odom_publisher.publish(msg);
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped tf_stamped;

    tf_stamped.header = msg.header;
    tf_stamped.child_frame_id = msg.child_frame_id;

    tf_stamped.transform.translation.x = msg.pose.pose.position.x;
    tf_stamped.transform.translation.y = msg.pose.pose.position.y;
    tf_stamped.transform.translation.z = msg.pose.pose.position.z;

    tf_stamped.transform.rotation.w = quat.w();
    tf_stamped.transform.rotation.x = quat.x();
    tf_stamped.transform.rotation.y = quat.y();
    tf_stamped.transform.rotation.z = quat.z();

    br.sendTransform(tf_stamped);

}

void Simulator::publish_diagnostics() {
    diagnostic_msgs::DiagnosticArray m;

    m.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus d;

    d.name = "thrusters";
    d.message = "Ok";
    d.hardware_id = "dynamics";

    diagnostic_msgs::KeyValue k;
    k.value = "true";
    k.key = "enabled";

    d.values.push_back(k);

    m.status.push_back(d);

    m_diagnostic_publisher.publish(m);
}

void Simulator::loop() {
    while (ros::ok()) {
        auto now = std::chrono::system_clock::now();
        iterate(g_controls);

        std::this_thread::sleep_until(now + std::chrono::duration<double>(m_dt));
        auto after = std::chrono::system_clock::now();
        if ((after - now) > std::chrono::duration<double>(m_dt + m_dt * 0.1)) {
            ROS_WARN("can not keep up with simulation rate %.3f, dt: %.5f", m_dt, std::chrono::duration<double>(after-now).count());
        }
    }
}