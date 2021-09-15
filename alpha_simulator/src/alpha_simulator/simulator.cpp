#include "simulator.h"
#include "state.h"
#include "cmath"
#include "utilities.hxx"
#include "thread"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

void Simulator::iterate(control_commands_t cmd) {

    g_thruster.XT = pwm2thrust(cmd.thruster_x);
    g_thruster.YT = pwm2thrust(cmd.thruster_y);
    g_thruster.ZT = pwm2thrust(cmd.thruster_z);

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
    g_inertia.ZG = (g_mass.stationary_battery_m * g_mass.stationary_battery_z +
                    g_mass.movable_battery_m * g_mass.movable_battery_z
                    + g_mass.hull_m * g_mass.hull_z) / g_mass.total;

    g_thruster.MT = -g_thruster.XT * g_inertia.ZG - g_thruster.ZT * (20 * 24.5 / 1000 - g_inertia.XG);
    g_thruster.NT = g_thruster.YT * (-20 * 25.4 / 1000 + g_inertia.XG);

    g_vehicle_state.u_dot = 1.0 / (g_mass.weight / 9.81 + g_hydro.added_mass[0]) *
                            (g_thruster.XT - g_hydro.damping[0] * g_vehicle_state.u * std::fabs(g_vehicle_state.u)
                             - g_mass.weight / 9.81 * (0 * g_vehicle_state.w - g_vehicle_state.r * g_vehicle_state.v
                                                       + 0 * g_inertia.ZG
                                                       - (0 + 0) * g_inertia.XG)
                             + (-g_mass.weight + g_mass.buoyancy) * std::sin(g_vehicle_state.pitch));


    g_vehicle_state.v_dot = 1.0 / (g_mass.weight / 9.81 + g_hydro.added_mass[1]) *
                            (g_thruster.YT - g_hydro.damping[1] * g_vehicle_state.v * std::fabs(g_vehicle_state.v)
                             - g_mass.weight / 9.81 * (g_vehicle_state.r * g_vehicle_state.u
                                                       + g_vehicle_state.r_dot * g_inertia.XG +
                                                       g_vehicle_state.r * g_inertia.ZG * g_vehicle_state.q));


    g_vehicle_state.w_dot = 1.0 / (g_mass.weight / 9.81 + g_hydro.added_mass[2]) *
                            (g_thruster.ZT - g_hydro.damping[2] * g_vehicle_state.w * std::fabs(g_vehicle_state.w)
                             - g_mass.weight / 9.81 *
                               (-g_vehicle_state.q * g_vehicle_state.u - g_vehicle_state.q_dot * g_inertia.XG -
                                std::pow(g_vehicle_state.q, 2) * g_inertia.ZG)
                             - (-g_mass.weight + g_mass.buoyancy) * std::cos(g_vehicle_state.pitch));


    g_vehicle_state.q_dot = 1.0 / (g_inertia.Iyy + g_hydro.added_mass[4]) *
                            (g_thruster.MT - g_hydro.damping[4] * g_vehicle_state.q * std::fabs(g_vehicle_state.q)
                             - g_inertia.Ixz * std::pow(g_vehicle_state.r, 2)
                             - g_mass.weight / 9.81 * (g_inertia.ZG *
                                                       (g_vehicle_state.u_dot + g_vehicle_state.q * g_vehicle_state.w -
                                                        g_vehicle_state.r * g_vehicle_state.v)
                                                       - g_inertia.XG * (g_vehicle_state.w_dot -
                                                                         g_vehicle_state.q * g_vehicle_state.u))
                             - g_inertia.ZG * g_mass.buoyancy * std::sin(g_vehicle_state.pitch) -
                             g_inertia.XG * g_mass.buoyancy * std::cos(g_vehicle_state.pitch));

    g_vehicle_state.r_dot = 1.0 / (g_inertia.Izz + g_hydro.added_mass[5]) *
                            (g_thruster.NT - g_hydro.damping[5] * g_vehicle_state.r * std::fabs(g_vehicle_state.r)
                             + g_inertia.Ixz * g_vehicle_state.q * g_vehicle_state.r
                             - g_mass.weight / 9.81 * g_inertia.XG *
                               (g_vehicle_state.v_dot + g_vehicle_state.r * g_vehicle_state.u));

    g_vehicle_state.u += m_dt * g_vehicle_state.u_dot;
    g_vehicle_state.v += m_dt * g_vehicle_state.v_dot;
    g_vehicle_state.w += m_dt * g_vehicle_state.w_dot;
    g_vehicle_state.q += m_dt * g_vehicle_state.q_dot;
    g_vehicle_state.r += m_dt * g_vehicle_state.r_dot;

    g_vehicle_state.pitch_dot = g_vehicle_state.q;
    g_vehicle_state.yaw_dot = std::cos(g_vehicle_state.pitch) * g_vehicle_state.r;
    g_vehicle_state.pitch = g_vehicle_state.pitch + m_dt * g_vehicle_state.pitch_dot;
    g_vehicle_state.yaw = g_vehicle_state.yaw + m_dt * g_vehicle_state.yaw_dot;


    // todo: use wrapTo2Pi like function if doesn't work
    // g_vehicle_state.pitch = std::atan2(std::cos(g_vehicle_state.pitch), std::sin(g_vehicle_state.pitch));
    // g_vehicle_state.yaw = std::atan2(std::cos(g_vehicle_state.yaw), std::sin(g_vehicle_state.yaw));

    // g_vehicle_state.pitch = wrapTo2Pi(g_vehicle_state.pitch);
    // g_vehicle_state.yaw = wrapTo2Pi(g_vehicle_state.yaw);

    g_vehicle_state.pitch = remainder(g_vehicle_state.pitch, 2 * M_PI);
    g_vehicle_state.yaw = remainder(g_vehicle_state.yaw, 2 * M_PI);

    Eigen::Matrix3d rbe = rotz(g_vehicle_state.yaw) * roty(g_vehicle_state.pitch);

    g_earth_state.x_dot = rbe.row(0) * Eigen::Vector3d{g_vehicle_state.u, g_vehicle_state.v, g_vehicle_state.w};
    g_earth_state.y_dot = rbe.row(1) * Eigen::Vector3d{g_vehicle_state.u, g_vehicle_state.v, g_vehicle_state.w};
    g_earth_state.z_dot = rbe.row(2) * Eigen::Vector3d{g_vehicle_state.u, g_vehicle_state.v, g_vehicle_state.w};

    g_earth_state.x = g_earth_state.x + g_earth_state.x_dot * m_dt;
    g_earth_state.y = g_earth_state.y + g_earth_state.y_dot * m_dt;
    g_earth_state.z = g_earth_state.z + g_earth_state.z_dot * m_dt;


    g_mass.W_pump = g_mass.W_pump + g_mass.v_pump * m_dt;
    if (g_mass.W_pump > 0.3) {
        g_mass.W_pump = 0.3;
    } else if (g_mass.W_pump < -0.3) {
        g_mass.W_pump = -0.3;
    }

    g_mass.movable_battery_x = g_mass.movable_battery_x + g_mass.movable_battery_vx * m_dt;
    if (g_mass.movable_battery_x > 3.25 * 25.4 / 1000) {
        g_mass.movable_battery_x = 3.25 * 25.4 / 1000;
    } else if (g_mass.movable_battery_x < -6.5 * 25.4 / 1000) {
        g_mass.movable_battery_x = -.6 * 25.4 / 1000;
    }

}

Simulator::Simulator() : m_nh() {

    m_dt = 0.01; //seconds

    m_odom_publisher = m_nh.advertise<nav_msgs::Odometry>("odom", 1000);

    m_cmd_subscriber = m_nh.subscribe("thrust_cmd", 100, &Simulator::cmd_callback, this);

    m_loop_thread = std::thread(std::bind(&Simulator::loop, this));
}

void Simulator::cmd_callback(const geometry_msgs::Point::ConstPtr &msg) {
    g_controls.thruster_x = msg->x;
    g_controls.thruster_y = msg->y;
    g_controls.thruster_z = msg->z;
}

void Simulator::publish_odometry() {

    nav_msgs::Odometry msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";

    msg.pose.pose.position.x = g_earth_state.x;
    msg.pose.pose.position.y = g_earth_state.y;
    msg.pose.pose.position.z = g_earth_state.z;

    tf2::Quaternion quat;
    quat.setRPY(g_vehicle_state.roll, g_vehicle_state.pitch, g_vehicle_state.yaw);
    msg.pose.pose.orientation.w = quat.w();
    msg.pose.pose.orientation.x = quat.x();
    msg.pose.pose.orientation.y = quat.y();
    msg.pose.pose.orientation.z = quat.z();

    msg.twist.twist.linear.x = g_vehicle_state.u;
    msg.twist.twist.linear.y = g_vehicle_state.v;
    msg.twist.twist.linear.z = g_vehicle_state.w;

    msg.twist.twist.angular.y = g_vehicle_state.q;
    msg.twist.twist.angular.z = g_vehicle_state.r;

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

void Simulator::loop() {
    while (ros::ok()) {
        auto now = std::chrono::system_clock::now();
        iterate(g_controls);

        publish_odometry();
        std::this_thread::sleep_until(now + std::chrono::duration<double>(m_dt));
        auto after = std::chrono::system_clock::now();
        if ((after - now) > std::chrono::duration<double>(m_dt + m_dt * 0.1)) {
            ROS_WARN("can not keep up with simulation rate %.3f, dt: %.5f", m_dt, std::chrono::duration<double>(after-now).count());
        }
    }
}