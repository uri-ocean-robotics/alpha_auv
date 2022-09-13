#include "state.hxx"
#include "cmath"
#include "Eigen/Dense"
#include "constants.h"

vehicle_state_t::vehicle_state_t() {
    u = 0;
    v = 0;
    w = 0;
    q = 0;
    r = 0;
    u_dot = 0;
    v_dot = 0;
    w_dot = 0;
    q_dot = 0;
    r_dot = 0;
    roll = 0;
    pitch = 0;
    yaw = 0;
    pitch_dot = 0;
    yaw_dot = 0;
    power = 0;
}

earth_state_t::earth_state_t() {

}

vehicle_dimension_t::vehicle_dimension_t() {
    L = 55 * INCH_TO_METER;
    r = 5 * INCH_TO_METER;
}

mass_t::mass_t() {
    hull_m = 16;
    stationary_battery_m = 3;
    movable_battery_m = 6;

    hull_x = 0;
    hull_y = 0;
    hull_z = - 0.5 * INCH_TO_METER;

    stationary_battery_x = -6.0 * INCH_TO_METER;
    stationary_battery_y = 0;
    stationary_battery_z = -2.25 * INCH_TO_METER;

    movable_battery_x = 3.0 * INCH_TO_METER;
    movable_battery_y = 0;
    movable_battery_z = 2.25 * INCH_TO_METER;
    movable_battery_vx = 0;

    total = hull_m + movable_battery_m + stationary_battery_m;

    v_pump = 0;
    W_pump = 0.1;

    buoyancy = total * 9.81 + W_pump * 9.81;
    weight = total * 9.81;
}

hydrodynamic_t::hydrodynamic_t() {
    mass = 25;
    r = 5 * INCH_TO_METER;
    L = 55 * INCH_TO_METER;

    rho = 1023;
    A11 = 0.1 * mass;
    A22 = rho * M_PI * std::pow(r, 2) * L;
    A33 = A22;
    A44 = 0;
    A55 = std::pow(L, 3) / 12 * rho * M_PI * std::pow(r, 2);
    A66 = A55;

    added_mass = {A11, A22, A33, A44, A55, A66};

    v = 0.75;

    kv = 1.267 / 1000000;

    Re = v * L / kv;
    Cd = 0.25;
    Cl = 0.94;
    Xuu = -1.0 / 2 * rho * Cd * M_PI * std::pow(r, 2);
    Yvv = -1.0 / 2 * rho * Cl * L * 2 * r;
    Zww = Yvv;
    Kpp =0;
    Mqq = -1.0 / 32 * Cl * r * std::pow(L, 4) * rho;
    Nrr = -1.0 / 32 * Cl * r * std::pow(L, 4) * rho;

    damping = {-Xuu, -Yvv, -Zww, -Kpp, -Mqq, -Nrr};

}

control_commands_t::control_commands_t() {
    thruster_x = 1500;
    thruster_y = 1500;
    thruster_z = 1500;
}

Eigen::Matrix3d transformation = Eigen::Matrix3d((Eigen::Matrix3d()
        << 1, 0, 0,
           0, -1, 0,
           0, 0, -1).finished());
earth_state_t g_world_state_ned = earth_state_t();
vehicle_state_t g_vehicle_state_ned = vehicle_state_t();

vehicle_dimension_t g_vehicle_dimensions = vehicle_dimension_t();
mass_t g_mass = mass_t();
inertial_t g_inertia = inertial_t();
thruster_t g_thruster = thruster_t();
hydrodynamic_t g_hydro = hydrodynamic_t();
control_commands_t g_controls = control_commands_t();

earth_state_t enu_to_ned(const earth_state_t enu) {

    earth_state_t ned;
    ned.point = enu.point.transpose() * transformation;
    ned.point_dot = enu.point_dot.transpose() * transformation;

    return ned;
}

vehicle_state_t enu_to_ned(const vehicle_state_t enu) {

    Eigen::Vector3d orientation;
    orientation << enu.roll, enu.pitch, enu.yaw;

    Eigen::Vector3d orientation_dot;
    orientation_dot << enu.roll_dot, enu.pitch_dot, enu.yaw_dot;

    Eigen::Vector3d linear_velocity;
    linear_velocity << enu.u, enu.v, enu.w;

    Eigen::Vector3d angular_velocity;
    angular_velocity << enu.p, enu.q, enu.r;

    Eigen::Vector3d angular_acceleration;
    angular_acceleration << enu.p_dot, enu.q_dot, enu.r_dot;


    auto orientation_ned = orientation.transpose() * transformation;
    auto orientation_dot_ned = orientation_dot.transpose() * transformation;
    auto linear_velocity_ned = linear_velocity.transpose() * transformation;
    auto angular_velocity_ned = angular_velocity.transpose() * transformation;
    auto angular_acceleration_ned = angular_acceleration.transpose() * transformation;

    vehicle_state_t ned;
    ned.roll = orientation_ned(0);
    ned.pitch = orientation_ned(1);
    ned.yaw = orientation_ned(2);

    ned.roll_dot = orientation_dot_ned(0);
    ned.pitch_dot = orientation_dot_ned(1);
    ned.yaw_dot = orientation_dot_ned(2);

    ned.u = linear_velocity_ned(0);
    ned.v = linear_velocity_ned(1);
    ned.w = linear_velocity_ned(2);

    ned.p = angular_velocity_ned(0);
    ned.q = angular_velocity_ned(1);
    ned.w = angular_velocity_ned(2);

    return ned;
}

earth_state_t ned_to_enu(const earth_state_t ned) {


    earth_state_t enu;
    enu.point = ned.point.transpose() * transformation.transpose();
    enu.point_dot = ned.point_dot.transpose() * transformation.transpose();

    return enu;
}

vehicle_state_t ned_to_enu(const vehicle_state_t ned) {
    Eigen::Vector3d orientation;
    orientation << ned.roll, ned.pitch, ned.yaw;

    Eigen::Vector3d orientation_dot;
    orientation_dot << ned.roll_dot, ned.pitch_dot, ned.yaw_dot;

    Eigen::Vector3d linear_velocity;
    linear_velocity << ned.u, ned.v, ned.w;

    Eigen::Vector3d angular_velocity;
    angular_velocity << ned.p, ned.q, ned.r;

    Eigen::Vector3d angular_acceleration;
    angular_acceleration << ned.p_dot, ned.q_dot, ned.r_dot;


    auto orientation_enu = orientation.transpose() * transformation.transpose();
    auto orientation_dot_enu = orientation_dot.transpose() * transformation.transpose();
    auto linear_velocity_enu = linear_velocity.transpose() * transformation.transpose();
    auto angular_velocity_enu = angular_velocity.transpose() * transformation.transpose();
    auto angular_acceleration_enu = angular_acceleration.transpose() * transformation.transpose();

    vehicle_state_t enu;
    enu.roll = orientation_enu(0);
    enu.pitch = orientation_enu(1);
    enu.yaw = orientation_enu(2);

    enu.roll_dot = orientation_dot_enu(0);
    enu.pitch_dot = orientation_dot_enu(1);
    enu.yaw_dot = orientation_dot_enu(2);

    enu.u = linear_velocity_enu(0);
    enu.v = linear_velocity_enu(1);
    enu.w = linear_velocity_enu(2);

    enu.p = angular_velocity_enu(0);
    enu.q = angular_velocity_enu(1);
    enu.w = angular_velocity_enu(2);

    return enu;
}
