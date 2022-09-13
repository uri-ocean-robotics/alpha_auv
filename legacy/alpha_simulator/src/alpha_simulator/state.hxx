#ifndef ALPHA_SIMULATOR_STATE_HXX
#define ALPHA_SIMULATOR_STATE_HXX

#include "vector"
#include "Eigen/Dense"

typedef struct vehicle_state_t {
    vehicle_state_t();
    double u;
    double v;
    double w;
    double p;
    double q;
    double r;
    double u_dot;
    double v_dot;
    double w_dot;
    double p_dot;
    double q_dot;
    double r_dot;
    double roll;
    double pitch;
    double yaw;
    double roll_dot;
    double pitch_dot;
    double yaw_dot;
    double power;
} vehicle_state_t;

typedef struct earth_state_t {
    earth_state_t();
    Eigen::Vector3d point;
    Eigen::Vector3d point_dot;
} earth_state_t;

typedef struct vehicle_dimension_t {
    vehicle_dimension_t();
    double L; //!< @brief: vehicle length
    double r; //!< @brief: vehicle radius
} vehicle_dimension_t;

typedef struct mass_t {
    mass_t();
    double total;
    double hull_m;
    double stationary_battery_m;
    double movable_battery_m;

    double hull_x;
    double hull_y;
    double hull_z;

    double stationary_battery_x;
    double stationary_battery_y;
    double stationary_battery_z;

    double movable_battery_x;
    double movable_battery_y;
    double movable_battery_z;
    double movable_battery_vx;

    double v_pump;
    double W_pump;

    double buoyancy;
    double weight;
    double variable_buoyancy;
} mass_t;

typedef struct inertial_t {
    inertial_t() = default;
    double Ixx;
    double Iyy;
    double Izz;
    double Ixy;
    double Ixz;
    double Iyz;
    double XG; //!< Center of gravity
    double YG; //!< Center of gravity
    double ZG; //!< Center of gravity
} inertial_t;

typedef struct thruster_t {
    thruster_t() = default;
    double Xpwm;
    double Ypwm;
    double Zpwm;
    double XT;
    double YT;
    double ZT;
    double MT;
    double NT;
} thruster_t;

typedef struct hydrodynamic_t {
    hydrodynamic_t();
    double mass;    //!< mass
    double r;       //!< radius
    double L;       //!< length

    double rho;     //!< water density;
    double A11;
    double A22;
    double A33;
    double A44;
    double A55;
    double A66;

    std::vector<double> added_mass;

    double v;       //!< damping terms v
    double kv;

    double Re;
    double Cd;
    double Cl;
    double Xuu;
    double Yvv;
    double Zww;
    double Kpp;
    double Mqq;
    double Nrr;

    std::vector<double> damping;
} hydrodynamic_t;


typedef struct control_commands_t {
    control_commands_t();
    double thruster_x;
    double thruster_y;
    double thruster_z;
} control_commands_t;

earth_state_t ned_to_enu(earth_state_t ned);
vehicle_state_t ned_to_enu(vehicle_state_t ned);

earth_state_t enu_to_ned(earth_state_t ned);
vehicle_state_t enu_to_ned(vehicle_state_t ned);

extern earth_state_t g_world_state_ned;
extern vehicle_state_t g_vehicle_state_ned;

// extern earth_state_t g_world_state_enu;
// extern vehicle_state_t g_vehicle_state_enu;

extern vehicle_dimension_t g_vehicle_dimensions;
extern mass_t g_mass;
extern inertial_t g_inertia;
extern thruster_t g_thruster;
extern hydrodynamic_t g_hydro;
extern control_commands_t g_controls;

extern Eigen::Matrix3d transformation;

#endif //ALPHA_SIMULATOR_STATE_HXX
