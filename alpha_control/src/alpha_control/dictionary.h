#ifndef ALPHA_CONTROL_DICTIONARY_H
#define ALPHA_CONTROL_DICTIONARY_H

#include "vector"
#include "string"

#define STATE_X             "x"
#define STATE_X_INDEX       0
#define STATE_Y             "y"
#define STATE_Y_INDEX       1
#define STATE_Z             "z"
#define STATE_Z_INDEX       2
#define STATE_ROLL          "roll"
#define STATE_ROLL_INDEX    3
#define STATE_PITCH         "pitch"
#define STATE_PITCH_INDEX   4
#define STATE_YAW           "yaw"
#define STATE_YAW_INDEX     5
#define STATE_U             "u"
#define STATE_U_INDEX       6
#define STATE_V             "v"
#define STATE_V_INDEX       7
#define STATE_W             "w"
#define STATE_W_INDEX       8

#define STATE_VECTOR_SIZE 9


#define CONF_THRUSTER_POLY "thruster_polynomials"
#define CONF_THRUST_COMMAND_TOPICS "thruster_command_topics"
#define CONF_THRUSTER_FORCE_TOPICS "thruster_force_topics"
#define CONF_THRUSTER_IDS "thruster_ids"

#define CONF_GENERATOR_TYPE "generator_type"
#define CONF_GENERATOR_TYPE_OPT_TF "tf"
#define CONF_GENERATOR_TYPE_OPT_USER "user"

#define CONF_TF_PREFIX "tf_prefix"
#define CONF_TF_PREFIX_DEFAULT ""
#define CONF_CG_LINK "cg_link"
#define CONF_CG_LINK_DEFAULT "cg_link"
#define CONF_WORLD_LINK "world_link"
#define CONF_WORLD_LINK_DEFAULT "world"
#define CONF_ODOMETRY_SOURCE "odometry_source"
#define CONF_ODOMETRY_SOURCE_DEFAULT "odometry"
#define CONF_CONTROL_MODES "control_modes"
#define CONF_CONTROL_RULES "control_rules"

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

#define TOPIC_CONTROL_STATE_CURRENT "control/state/current"
#define TOPIC_CONTROL_STATE_DESIRED "control/state/desired"
#define TOPIC_CONTROL_STATE_ERROR "control/state/error"

#define SERVICE_GET_CONTROL_RULES "control/get_rules"
#define SERVICE_SET_CONTROL_POINT "control/set_point"


static const char * const STATES[] = {
        STATE_X,
        STATE_Y,
        STATE_Z,
        STATE_ROLL,
        STATE_PITCH,
        STATE_YAW,
        STATE_U,
        STATE_V,
        STATE_W,
        nullptr
};

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
