# 关节指令输出
std_msgs/Header header

# 关节名称
string[] joint_names

# 目标关节角度 (rad)
float64[] positions

# 目标关节速度 (rad/s)
float64[] velocities

# 前馈力矩 (Nm)
float64[] torques

# PD增益
float64[] kp
float64[] kd
