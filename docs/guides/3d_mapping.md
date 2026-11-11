# 3D 建图（LIO-SAM → PCD → 2D 栅格 → 导航）

> 目的：让 3D 定位有配套地图，形成 **建图 → 定位 → 导航** 的完整闭环。
> 状态：**已跑通**（2026-11-04，Jazzy + gz sim 8）。

## 1. 四步流程

```bash
# 环境
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export PATH=/opt/ros/jazzy/opt/gz_tools_vendor/bin:$PATH
export ROS_DOMAIN_ID=80 GZ_PARTITION=pnc_nav_map

# ① 起 LIO-SAM（建图模式：savePCD=true）
ros2 launch pnc_nav_bringup gz_sim_3d_lidar_bringup.launch.py \
  localization:=lio_sam world:=lio_room.sdf \
  lio_params:=$HOME/pnc_nav2/src/pnc_nav_sim/params/lio_sam_sim3d_mapping.yaml

# ② 开着机器人走一圈（覆盖要建的区域）
python3 test/lio_drift_test.py --laps 1 --forward 4.0 --side 3.0 \
  --gt-topic /world/lio_room/dynamic_pose/info --out /tmp/mapping_run

# ③ 存 PCD 地图（destination 相对 $HOME）
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap \
  "{resolution: 0.1, destination: '/pnc_maps/lio_room'}"

# ④ PCD → 2D 栅格（PGM + YAML，nav2_map_server 可直接读）
python3 scripts/pcd_to_map.py \
  --pcd ~/pnc_maps/lio_room/GlobalMap.pcd \
  --out ~/pnc_maps/lio_room/map --resolution 0.05 --z-min 0.15 --z-max 1.5

# ⑤ 用自建地图跑导航
ros2 launch pnc_nav_bringup gz_sim_3d_lidar_bringup.launch.py \
  localization:=lio_sam world:=lio_room.sdf map:=$HOME/pnc_maps/lio_room/map.yaml
python3 test/action_smoke_test.py feedback 2.0 2.0
```

## 2. 实测结果（lio_room，12×8 m 房间）

| 环节 | 结果 |
|---|---|
| PCD 保存 | `CornerMap.pcd` 35K / `SurfMap.pcd` 237K / `GlobalMap.pcd` 1014K（共 6.5 万点） |
| 转栅格 | 260×179 @0.05m，origin=(-4.52,-4.46)，占用格 8.5% |
| 用它导航 | 目标 (2.0, 2.0)：**success，24.38s / 2.85m** |

## 3. `pcd_to_map.py` 说明

| 参数 | 说明 |
|---|---|
| `--pcd` | 可多指定（例如 CornerMap + SurfMap 合并） |
| `--resolution` | 栅格分辨率，默认 0.05 |
| `--z-min/--z-max` | **相对地面**的高度带（默认 0.15–1.5m）：过滤地面与天花板 |
| `--pad` | 地图外扩边距 |
| `--dilate` | 障碍膨胀格数（默认 1） |

实现要点：
- 地面高度用点云 z 的 **5% 分位**估计（LIO-SAM 的 map 原点在雷达起始位置，z 不是 0，实测地面 ≈ -0.44m）
- PCD 读取支持 `DATA binary` / `DATA ascii`（`binary_compressed` 需先转换）
- 输出 PGM 行序翻转（PGM 从上到下），YAML 带 origin/resolution/threshold

## 4. 已知限制

| 项 | 说明 |
|---|---|
| 静态 2D 图 | 只保留"一层"占用信息；多层/悬空结构（桌子、斜坡下方）会被压平 |
| OctoMap | 未接（`ros-jazzy-octomap-server` 不在本机 apt 源）；需要 3D 占据图时再接 |
| 回环 | 建图时 LIO-SAM 回环默认关（`lio_sam_sim3d.yaml`），长走廊会有累积误差 |
| 地图坐标系 | 与 LIO-SAM 的世界原点（启动位置）一致；换起点建图需要重新对齐 |

## 5. 相关文件

```
src/pnc_nav_sim/params/lio_sam_sim3d_mapping.yaml   # savePCD=true 的建图参数
scripts/pcd_to_map.py                                # PCD → PGM/YAML
test/lio_drift_test.py                               # 也可当"走一圈"的驱动脚本
docs/guides/lio_sam_setup.md                         # LIO-SAM 安装/改造
```

## 6. 3D 占据栅格（OctoMap）

```bash
# 需要：sudo apt install ros-jazzy-octomap-server
# 前提：正在跑 LIO-SAM（点云话题 /lio_sam/mapping/cloud_registered）
ros2 launch pnc_nav_map octomap.launch.py           # 默认 cloud_topic=/lio_sam/mapping/cloud_registered
```

| 输出 | 说明 |
|---|---|
| `/octomap_binary` | 3D 占据栅格（二进制概率），frame=map |
| `/octomap_full` | 完整概率体素 |
| `/projected_map` | 投影到 2D 的 OccupancyGrid（可直接给 nav2 当全局图） |

配置在 `src/pnc_nav_map/config/octomap.yaml`：分辨率 0.05m、高度带 -1.0~2.5m、
`latch: true`（建完保持发布）、`publish_free_space: false`（省带宽）。

**一个坑**：LIO-SAM 的配准点云在 `odom` 帧，而 octomap 默认目标帧是 `map`，
我们的 LIO-SAM 改造只发 `map→base_link`（不发 `map→odom`）→ 消息会被全部丢弃
（`Message Filter dropping message: frame 'odom'`）。
`octomap.launch.py` 里补了一个**恒等 `map→odom` 静态 TF**（`publish_map_to_odom:=true`，可关）解决。

实测：`/octomap_binary` 0.05m 分辨率、124 KB、frame=map，无丢弃告警。

