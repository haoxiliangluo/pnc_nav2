# 导航服务器状态外发（给任务层/HMI/监控用；action feedback 仍然保留）
std_msgs/Header header

uint8 state
uint8 IDLE=0
uint8 PLANNING=1
uint8 FOLLOWING=2
uint8 RECOVERING=3
uint8 SUCCEEDED=4
uint8 FAILED=5

uint16 error_code            # 最近一次结束的错误码（0=成功，见 docs/design/nav_server.md）
float64 distance_remaining   # 到目标直线距离 (m)；空闲时为 0
uint16 recoveries            # 本次会话已恢复次数
float64 total_time           # 本次会话已耗时 (s)
string message               # 人类可读状态描述
