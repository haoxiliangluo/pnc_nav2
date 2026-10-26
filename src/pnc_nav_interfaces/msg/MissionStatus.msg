# 任务执行状态（pnc_nav_task）
std_msgs/Header header

uint8 state
uint8 IDLE=0
uint8 RUNNING=1
uint8 PAUSED=2
uint8 SUCCEEDED=3
uint8 FAILED=4
uint8 CANCELLED=5

uint16 current_index         # 当前点序号（1-based；0=还未开始）
uint16 total                 # 总点数
uint16 retries_done          # 当前点已重试次数
geometry_msgs/PoseStamped current_waypoint
string message
