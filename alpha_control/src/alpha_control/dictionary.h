#ifndef ALPHA_CONTROL_DICTIONARY_H
#define ALPHA_CONTROL_DICTIONARY_H

#include "vector"
#include "string"
#include "map"

#include "alpha_control/ControlModes.h"

static constexpr const char * STATE_X = "x";
static constexpr const char * STATE_Y = "y";
static constexpr const char * STATE_Z = "z";
static constexpr const char * STATE_ROLL = "roll";
static constexpr const char * STATE_PITCH = "pitch";
static constexpr const char * STATE_YAW = "yaw";
static constexpr const char * STATE_SURGE = "surge";
static constexpr const char * STATE_SWAY = "sway";
static constexpr const char * STATE_HEAVE = "heave";

static constexpr int STATE_X_INDEX = alpha_control::ControlMode::DOF_X;
static constexpr int STATE_Y_INDEX = alpha_control::ControlMode::DOF_Y;
static constexpr int STATE_Z_INDEX = alpha_control::ControlMode::DOF_Z;
static constexpr int STATE_ROLL_INDEX = alpha_control::ControlMode::DOF_ROLL;
static constexpr int STATE_PITCH_INDEX = alpha_control::ControlMode::DOF_PITCH;
static constexpr int STATE_YAW_INDEX = alpha_control::ControlMode::DOF_YAW;
static constexpr int STATE_SURGE_INDEX = alpha_control::ControlMode::DOF_SURGE;
static constexpr int STATE_SWAY_INDEX = alpha_control::ControlMode::DOF_SWAY;
static constexpr int STATE_HEAVE_INDEX = alpha_control::ControlMode::DOF_HEAVE;

static const std::map<const char *, int> STATE_IDX = {
  {STATE_X,     alpha_control::ControlMode::DOF_X },
  {STATE_Y,     alpha_control::ControlMode::DOF_Y },
  {STATE_Z,     alpha_control::ControlMode::DOF_Z },
  {STATE_ROLL,  alpha_control::ControlMode::DOF_ROLL },
  {STATE_PITCH, alpha_control::ControlMode::DOF_PITCH },
  {STATE_YAW,   alpha_control::ControlMode::DOF_YAW },
  {STATE_SURGE, alpha_control::ControlMode::DOF_SURGE },
  {STATE_SWAY,  alpha_control::ControlMode::DOF_SWAY },
  {STATE_HEAVE, alpha_control::ControlMode::DOF_HEAVE },
};

static const char * const STATES[] = {
        STATE_X,
        STATE_Y,
        STATE_Z,
        STATE_ROLL,
        STATE_PITCH,
        STATE_YAW,
        STATE_SURGE,
        STATE_SWAY,
        STATE_HEAVE,
        nullptr
};

#define STATE_VECTOR_SIZE 9


static constexpr const char * CONF_THRUSTER_POLY = "thruster_polynomials";
static constexpr const char * CONF_THRUST_COMMAND_TOPICS = "thruster_command_topics";
static constexpr const char * CONF_THRUSTER_FORCE_TOPICS = "thruster_force_topics";
static constexpr const char * CONF_THRUSTER_IDS = "thruster_ids";

static constexpr const char * CONF_GENERATOR_TYPE = "generator_type";
static constexpr const char * CONF_GENERATOR_TYPE_OPT_TF = "tf";
static constexpr const char * CONF_GENERATOR_TYPE_OPT_USER = "user";

static constexpr const char * CONF_TF_PREFIX = "tf_prefix";
static constexpr const char * CONF_TF_PREFIX_DEFAULT = "";
static constexpr const char * CONF_CG_LINK = "cg_link";
static constexpr const char * CONF_CG_LINK_DEFAULT = "cg_link";
static constexpr const char * CONF_WORLD_LINK = "world_link";
static constexpr const char * CONF_WORLD_LINK_DEFAULT = "world";
static constexpr const char * CONF_ODOMETRY_SOURCE = "odometry_source";
static constexpr const char * CONF_ODOMETRY_SOURCE_DEFAULT = "odometry";
static constexpr const char * CONF_CONTROL_MODES = "control_modes";

#define CONF_ENABLED "enabled"

#define CONF_PID "pid"
#define CONF_PID_P "p"
#define CONF_PID_P_INDEX 0
#define CONF_PID_I "i"
#define CONF_PID_I_INDEX 1
#define CONF_PID_D "d"
#define CONF_PID_D_INDEX 2
#define CONF_PID_I_MAX "i_max"
#define CONF_PID_I_MAX_INDEX 3
#define CONF_PID_I_MIN "i_min"
#define CONF_PID_I_MIN_INDEX 4

#define CONF_PID_GAINS_SIZE 5

#define CONF_PID_DEFAULT_ANY 0

#define CONF_CONTROL_ALLOCATION_MATRIX "control_allocation_matrix"
#define CONF_CONTROL_TF "control_tf"

#define TOPIC_SAFETY "safety"
#define TOPIC_STATUS "status"
#define TOPIC_CONTROL_STATE_CURRENT "controller/state/current"
#define TOPIC_CONTROL_STATE_DESIRED "controller/state/desired"
#define TOPIC_CONTROL_STATE_ERROR "controller/state/error"

#define SERVICE_CONTROL_ENABLE "controller/enable"
#define SERVICE_CONTROL_DISABLE "controller/disable"
#define SERVICE_GET_CONTROL_RULES "controller/get_rules"
#define SERVICE_SET_CONTROL_POINT "controller/set_point"



static const char * const CONF_PID_GAINS[] = {
        CONF_PID_P,
        CONF_PID_I,
        CONF_PID_D,
        CONF_PID_I_MAX,
        CONF_PID_I_MIN,
        nullptr
};


#define THRUST_LIMIT_NEWTON 20

#endif //ALPHA_CONTROL_DICTIONARY_H
