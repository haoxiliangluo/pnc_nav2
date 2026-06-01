# 规划器状态
std_msgs/Header header
string planner_name
uint8 status

# 状态常量
uint8 STATUS_IDLE=0
uint8 STATUS_PLANNING=1
uint8 STATUS_SUCCEEDED=2
uint8 STATUS_FAILED=3

# 规划统计
float64 planning_time_ms
uint32 path_length_points
float64 path_length_meters
