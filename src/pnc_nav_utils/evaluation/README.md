# pnc_nav_utils

导航系统调试与测试工具集。

## 快速开始

**重要：所有命令都应从项目根目录 `/home/hao/pnc_nav2/` 运行**

```bash
# 1. 打开代价地图编辑器，画障碍物
cd /home/hao/pnc_nav2
python3 src/pnc_nav_utils/evaluation/costmap_editor.py

# 2. 启动导航服务器（终端1）
ros2 launch pnc_nav_bringup nav_bringup.launch.py

# 3. 测试 C++ 规划器（终端2）
python3 src/pnc_nav_utils/evaluation/planner_tester.py

# 4. 评估导航性能（可选）
python3 src/pnc_nav_utils/evaluation/nav_evaluator.py

# 5. 监测单次导航质量（推荐）
python3 src/pnc_nav_utils/evaluation/nav_quality_monitor.py

# 6. 记录 RViz 2D Nav Goal 点（用于固定点复测）
python3 src/pnc_nav_utils/evaluation/goal_recorder.py
```

---

## 前置依赖

Python 依赖：
```bash
pip install numpy matplotlib pyyaml
```

ROS2 依赖：`rclpy`, `nav_msgs`, `geometry_msgs`, `tf2_ros`

---

## 工具详解

### 1. costmap_editor.py

**功能**：交互式代价地图编辑器，左键拖拽画矩形障碍物，右键删除，导出为 YAML。

**用法**：
```bash
cd /home/hao/pnc_nav2
python3 src/pnc_nav_utils/evaluation/costmap_editor.py

# 自定义地图参数
python3 src/pnc_nav_utils/evaluation/costmap_editor.py \
  --origin -5 -5 --size 15 15 --resolution 0.05
```

**参数**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--origin x y` | `-5 -5` | 地图原点 (m) |
| `--size w h` | `10 10` | 地图尺寸 (m) |
| `--resolution` | `0.1` | 栅格分辨率 (m) |
| `--robot-radius` | `0.25` | 致命半径 (m) |
| `--inflation-radius` | `0.5` | 膨胀半径 (m) |

**交互**：

| 操作 | 功能 |
|------|------|
| 左键拖拽 | 画矩形障碍物 |
| 右键点击 | 删除最近的障碍物 |
| `s` | 导出障碍物到 `costmap_obstacles.yaml` |
| `c` | 清空所有障碍物 |
| `r` | 重绘 |
| `q` | 退出 |

**导出示例**：

导出的 `costmap_obstacles.yaml` 格式：
```yaml
# 矩形障碍物: [min_x, min_y, max_x, max_y, ...]
obstacles: [
  0.50, 1.95, 3.50, 2.05,
  -1.05, -2.00, -0.95, 2.00,
]
```

将此内容粘贴到 `src/pnc_nav_bringup/config/nav_params.yaml` 的 `costmap.obstacles` 字段。

---

### 2. planner_tester.py

**功能**：C++ 全局规划器可视化测试工具。通过 ROS2 话题与 NavServer 交互，调用 C++ AStar2D 规划器并在 matplotlib 中显示结果。

**工作流程**：

1. 发布假 TF (`map` → `base_link`) 设定机器人起点
2. 发布 `goal_pose` 话题触发 NavServer 调用 C++ 规划器
3. 订阅 `global_plan` 话题获取规划路径
4. matplotlib 可视化代价地图 + 路径

**用法**：

```bash
cd /home/hao/pnc_nav2

# 终端1: 启动 NavServer
ros2 launch pnc_nav_bringup nav_bringup.launch.py

gdb -ex run --args /home/hao/pnc_nav2/install/pnc_nav_core/lib/pnc_nav_core/nav_server_node     --ros-args     --params-file /home/hao/pnc_nav2/src/pnc_nav_bringup/config/nav_params.yaml     -p use_sim_time:=false


# 终端2: 启动测试工具（自动查找配置文件）
python3 src/pnc_nav_utils/evaluation/planner_tester.py

# 使用 costmap_editor 导出的障碍物（只影响 Python 侧可视化）
python3 src/pnc_nav_utils/evaluation/planner_tester.py \
  --obstacles-yaml costmap_obstacles.yaml
```

**参数**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--config` | 自动查找 | nav_params.yaml 路径（可选） |
| `--obstacles-yaml` | 无 | costmap_editor 导出的障碍物文件（覆盖 config 中的 obstacles，仅影响可视化） |

**交互**：

| 操作 | 功能 |
|------|------|
| 左键点击 | 设置 start 点（绿色三角），发布 TF |
| 右键点击 | 设置 goal 点（红色星），触发 C++ 规划器 |
| `c` | 清除路径和 start/goal |
| `q` / 关闭窗口 / `Ctrl+C` | 退出 |

**注意事项**：

- `--obstacles-yaml` **只影响 Python 侧可视化**，C++ 规划器仍使用 NavServer 从 `nav_params.yaml` 读取的障碍物
- 如需 C++ 侧也使用编辑器画的障碍物，需将导出内容粘贴到 `nav_params.yaml` 的 `costmap.obstacles` 字段并**重启 NavServer**

---

### 3. nav_evaluator.py

**功能**：导航性能评估工具，在线模式实时评估导航性能。

**评估指标**：

- 路径长度 (m)
- 路径平滑度（曲率积分）
- 实际行驶距离 (m)
- 总导航时间 (s)
- 高度变化 (m)

**用法**：

```bash
cd /home/hao/pnc_nav2

# 在 NavServer 运行时启动评估工具
python3 src/pnc_nav_utils/evaluation/nav_evaluator.py

# 运行导航任务...
# 按 Ctrl+C 退出并保存结果到 nav_benchmark.csv
```

**输出**：

生成 `nav_benchmark.csv` 文件，包含每次导航的评估指标：

```csv
trial,path_length,actual_distance,total_time_s,smoothness,height_change
0,12.345,12.567,8.2,0.4523,0.125
1,15.678,15.892,10.5,0.5124,0.234
```

---

### 4. nav_quality_monitor.py

**功能**：导航效果验收脚本。它会在线监听目标点、全局路径、可选局部路径、里程计和速度指令，自动判断一次导航是否成功，并给出路径跟踪误差、控制平滑性和总评分。

**监听话题**：

| 话题 | 默认值 | 说明 |
|------|--------|------|
| goal | `/goal_pose` | RViz 或任务层发布的目标点 |
| global plan | `/global_plan` | 全局规划路径 |
| local plan | `/local_plan` | 局部规划路径，可选；没有也能运行 |
| odom | `/odom` | 实际轨迹来源 |
| cmd_vel | `/cmd_vel` | 控制输出平滑性评估来源 |

**核心指标**：

- 是否到达目标、总耗时、最终目标误差
- 全局路径长度、实际行驶距离、路径效率
- 实际轨迹相对全局路径的平均误差、RMSE、P95、最大误差
- 如果存在 `/local_plan`，额外统计局部路径相对全局路径、实际轨迹相对局部路径的偏差
- 平均/最大线速度、平均/最大角速度、控制抖动分数、停顿次数
- `navigation_score`：0-100 的综合评分

**用法**：

```bash
cd /home/hao/pnc_nav2
source install/setup.bash

python3 src/pnc_nav_utils/evaluation/nav_quality_monitor.py
```

常用参数：

```bash
python3 src/pnc_nav_utils/evaluation/nav_quality_monitor.py \
  --goal-tolerance 0.2 \
  --timeout 60 \
  --print-period 1.0 \
  --plot \
  --csv-output nav_quality_results.csv \
  --json-output nav_quality_latest.json
```

不加 `--plot` 时，脚本仍会按 `--print-period` 周期输出运行状态，例如目标误差、实际行驶距离、最近跟踪 RMSE 和当前速度指令。
加 `--plot` 后，脚本启动时会先打开一个空的 Matplotlib 窗口等待 `/goal_pose`，收到目标点和路径/里程计后实时绘制：

- 蓝线：`/global_plan`
- 橙线：`/local_plan`，如果存在
- 绿线：`/odom` 实际轨迹
- 红星：目标点

如果当前 Matplotlib 默认使用 `agg` 这类非 GUI backend，脚本会自动尝试切换到 `TkAgg` 或 Qt backend。仍无法打开窗口时，评价统计会继续运行，但 live plot 会被禁用；可尝试：

```bash
MPLBACKEND=TkAgg python3 src/pnc_nav_utils/evaluation/nav_quality_monitor.py --plot
```

输出文件：

```text
nav_quality_results.csv   # 多次 trial 汇总
nav_quality_latest.json   # 最近一次 trial 详情
```

---

### 5. goal_recorder.py

**功能**：监听 RViz `2D Nav Goal` 发布的 `/goal_pose`，把每次点击的目标点追加写入 CSV 和 JSONL。若 `/odom` 可用，会同时记录收到 goal 时最近一次里程计位置，作为该次测试的 start 参考。

**用法**：

```bash
cd /home/hao/pnc_nav2
source install/setup.bash

python3 src/pnc_nav_utils/evaluation/goal_recorder.py \
  --csv-output test/recorded_goals.csv \
  --jsonl-output test/recorded_goals.jsonl
```

**输出字段**：

| 字段 | 说明 |
|------|------|
| `goal_id` | 本次记录编号 |
| `goal_x`, `goal_y`, `goal_z` | RViz 目标点位置 |
| `goal_yaw_rad` | RViz 目标朝向 |
| `goal_qx/qy/qz/qw` | 原始四元数 |
| `start_available` | 是否记录到 odom 起点 |
| `start_x`, `start_y`, `start_z`, `start_yaw_rad` | 收到 goal 时最近一次 odom 位姿 |

---

## 典型工作流

```
1. costmap_editor 画障碍物 → 导出 costmap_obstacles.yaml
2. 将导出内容粘贴到 nav_params.yaml 的 costmap.obstacles 字段
3. 启动 NavServer（读取 nav_params.yaml 中的 obstacles）
4. planner_tester 测试 C++ 规划器
5. nav_quality_monitor 验收导航效果并记录评分
```

---

## 故障排除

**问题：`找不到配置文件 src/pnc_nav_bringup/config/nav_params.yaml`**

解决：确保从项目根目录 `/home/hao/pnc_nav2/` 运行脚本，或使用 `--config` 参数指定绝对路径。

**问题：planner_tester 窗口显示代价地图，但右键点击没反应**

解决：检查 NavServer 是否正常运行，使用 `ros2 topic list` 查看 `goal_pose` 和 `global_plan` 话题是否存在。

**问题：nav_evaluator 没有收到数据**

解决：检查话题名称是否正确（`global_plan`, `odom`），使用 `ros2 topic echo /global_plan` 验证话题有数据。
