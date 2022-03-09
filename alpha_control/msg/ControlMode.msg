int32 DOF_X     = 0
int32 DOF_Y     = 1
int32 DOF_Z     = 2
int32 DOF_ROLL  = 3
int32 DOF_PITCH = 4
int32 DOF_YAW   = 5
int32 DOF_SURGE = 6
int32 DOF_SWAY  = 7
int32 DOF_HEAVE = 8

string name
int32[] dofs

alpha_control/PIDgains pid_x
alpha_control/PIDgains pid_y
alpha_control/PIDgains pid_z

alpha_control/PIDgains pid_roll
alpha_control/PIDgains pid_pitch
alpha_control/PIDgains pid_yaw

alpha_control/PIDgains pid_surge
alpha_control/PIDgains pid_sway
alpha_control/PIDgains pid_heave