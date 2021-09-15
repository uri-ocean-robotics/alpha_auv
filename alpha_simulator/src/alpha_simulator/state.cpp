#include "state.h"
#include "cmath"

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
    x = 0;
    y = 0;
    z = 0;
    x_dot = 0;
    y_dot = 0;
    z_dot = 0;
}

vehicle_dimension_t::vehicle_dimension_t() {
    L = 55 * 25.4 / 1000;
    r = 5 * 25.4 / 1000;
}

mass_t::mass_t() {
    hull_m = 16;
    stationary_battery_m = 3;
    movable_battery_m = 6;

    hull_x = 0;
    hull_y = 0;
    hull_z = - 0.5 * 25.4 / 1000;

    stationary_battery_x = -6.0 * 25.4 / 1000;
    stationary_battery_y = 0;
    stationary_battery_z = -2.25 * 25.4 / 1000;

    movable_battery_x = 3.0 * 25.4 / 1000;
    movable_battery_y = 0;
    movable_battery_z = 2.25 * 25.4 / 1000;
    movable_battery_vx = 0;

    total = hull_m + movable_battery_m + stationary_battery_m;

    v_pump = 0;
    W_pump = 0;

    buoyancy = total * 9.81 + W_pump * 9.81;
    weight = total * 9.81;
}

hydrodynamic_t::hydrodynamic_t() {
    mass = 15;
    r = 5 * 25.4 / 1000;
    L = 55 * 25.4 / 1000;

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

earth_state_t g_earth_state = earth_state_t();
vehicle_state_t g_vehicle_state = vehicle_state_t();
vehicle_dimension_t g_vehicle_dimensions = vehicle_dimension_t();
mass_t g_mass = mass_t();
inertial_t g_inertia = inertial_t();
thruster_t g_thruster = thruster_t();
hydrodynamic_t g_hydro = hydrodynamic_t();
control_commands_t g_controls = control_commands_t();

