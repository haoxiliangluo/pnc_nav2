# 机器人状态（用于步态控制器）
std_msgs/Header header

# 基座姿态 (RPY)
float64 roll
float64 pitch
float64 yaw

# 基座角速度
geometry_msgs/Vector3 angular_velocity

# 基座线速度
geometry_msgs/Vector3 linear_velocity

# 关节状态 (12 joints: 3 per leg × 4 legs)
# 顺序: FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf,
#       RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf
float64[] joint_positions
float64[] joint_velocities
float64[] joint_torques

# 足端接触状态 (4 legs: FL, FR, RL, RR)
bool[] foot_contacts

# 足端位置 (相对于基座)
geometry_msgs/Point[] foot_positions
