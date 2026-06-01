# 可通行性地图
std_msgs/Header header
float64 resolution          # 地图分辨率 (m)
float64 origin_x            # 地图原点x
float64 origin_y            # 地图原点y
uint32 width                # 地图宽度 (cells)
uint32 height               # 地图高度 (cells)
float32[] traversability    # 可通行性值 [0,1], row-major
float32[] elevation         # 高程值 (m)
float32[] slope             # 坡度 (rad)
float32[] roughness         # 粗糙度
