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

## 典型工作流

```
1. costmap_editor 画障碍物 → 导出 costmap_obstacles.yaml
2. 将导出内容粘贴到 nav_params.yaml 的 costmap.obstacles 字段
3. 启动 NavServer（读取 nav_params.yaml 中的 obstacles）
4. planner_tester 测试 C++ 规划器
5. nav_evaluator 评估导航性能（可选）
```

---

## 故障排除

**问题：`找不到配置文件 src/pnc_nav_bringup/config/nav_params.yaml`**

解决：确保从项目根目录 `/home/hao/pnc_nav2/` 运行脚本，或使用 `--config` 参数指定绝对路径。

**问题：planner_tester 窗口显示代价地图，但右键点击没反应**

解决：检查 NavServer 是否正常运行，使用 `ros2 topic list` 查看 `goal_pose` 和 `global_plan` 话题是否存在。

**问题：nav_evaluator 没有收到数据**

解决：检查话题名称是否正确（`global_plan`, `odom`），使用 `ros2 topic echo /global_plan` 验证话题有数据。
