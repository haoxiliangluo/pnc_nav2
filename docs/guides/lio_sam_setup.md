# LIO-SAM 3D 定位接入

> 目标：用激光 + IMU 的 LIO-SAM 替代"固定 map→odom"，为 3D 场景提供真实定位来源。
> 状态：**已验证可用**（Jazzy + gz sim 8，2026-11-04）。

## 1. 一句话用法

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export PATH=/opt/ros/jazzy/opt/gz_tools_vendor/bin:$PATH
export LD_LIBRARY_PATH=/opt/gtsam_sys/lib:$LD_LIBRARY_PATH

ros2 launch pnc_nav_bringup gz_sim_3d_lidar_bringup.launch.py \
  localization:=lio_sam use_gui:=false use_rviz:=false
```

启动后：

| 输出 | 说明 |
|---|---|
| TF `map → base_link` | LIO-SAM 优化后的世界位姿（本框架 nav_server 直接用它） |
| `/lio_sam/mapping/odometry` | 位姿（BEST_EFFORT QoS，订阅端要匹配） |
| `/lio_sam/mapping/cloud_registered` | 配准到地图的点云 |
| `/odometry/imu` | IMU 预积分里程计（仅话题，**不发 TF**） |

## 2. 环境准备（一次性）

```bash
./scripts/setup_lio_sam.sh     # 装依赖 + 源码编 GTSAM（含 unstable）+ 处理 apt 包坑
export CMAKE_PREFIX_PATH=/opt/gtsam_sys:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=/opt/gtsam_sys/lib:$LD_LIBRARY_PATH
colcon build --packages-select lio_sam
```

踩过的坑（脚本已自动处理）：

| 现象 | 原因 | 处理 |
|---|---|---|
| `does not exist: libCppUnitLite.a` | apt 的 libgtsam-dev 打包不全 | 建空 `ar` 占位库 |
| `gtsam_unstable/...IncrementalFixedLagSmoother.h: 没有那个文件` | apt 无 unstable 分发包 | 源码编 GTSAM `-DGTSAM_BUILD_UNSTABLE=ON` |
| `static assertion failed: GTSAM was built against a different version of Eigen` | GTSAM 默认用自带 Eigen | `-DGTSAM_USE_SYSTEM_EIGEN=ON` 重编 |
| `没有规则可制作目标 libmetis.so` | 缺 dev 符号链接 | `apt install libmetis-dev` |
| CMake 找到 `/usr/lib/cmake/GTSAM`（无 unstable） | 路径优先级 | 把该目录改名 `GTSAM.disabled`（脚本 step 4） |
| `Eigen` / `Eigen3` 找不到 | 上游 CMakeLists 用老式 `find_package(Eigen)` | patch 为 `Eigen3` 并从 `ament_target_dependencies` 移除 |

> ⚠️ `setup_lio_sam.sh` 会修改系统路径（`/usr/lib/cmake/GTSAM` → `.disabled`）。换机器重跑脚本即可。

## 3. 我们对上游 LIO-SAM 的改造（`third_party/LIO-SAM`）

上游 LIO-SAM 的 TF 输出在 Nav2 类框架里会打架，做了两处最小改动：

1. **`mapOptmization.cpp`：发布 `map → base_link`**
   - 上游：`odom → lidar_link`（父帧是 odometryFrame，子帧写死 "lidar_link"）
   - 现在：`mapFrame → baselinkFrame`，用参数 `lidarToBase`（默认 `[0,0,-0.3]`，雷达在 base 上方 0.3m）把雷达位姿换成 base 位姿
   - 原因：nav_server / 代价地图要 `map→base_link`；同时避免与 URDF 的 `base_link→velodyne` 出现双父帧
2. **`imuPreintegration.cpp`：不再发 IMU 预积分 TF**（`publishImuTf` 默认 false，位姿仍走 `/odometry/imu` 话题）
   - 原因：否则 `map→base_link`（优化后）与 `odom→base_link`（IMU 积分、会漂）同时存在，TF 树冲突

另外仿真侧配合：

- `diff_drive_3d.urdf.xacro` 增加 gz `imu` 传感器（200 Hz，挂 `base_link`）
- `pnc_nav_sim/params/bridge_3d_lio.yaml`：**不桥 `tf`**（否则 gz 的 `odom→base_link` 与 LIO-SAM 冲突），并桥 `imu`
- `pnc_nav_sim/params/lio_sam_sim3d.yaml`：16 线参数（`N_SCAN=16`、`Horizon_SCAN=1800`、`sensor=velodyne`）、`lidarToBase`

## 4. 验证结果（2026-11-04）

平地直线前进 0.75 m（真值来自 gz `/odom`）：

| 来源 | x (m) | y (m) | 与真值误差 |
|---|---|---|---|
| gz 真值 `/odom` | 0.75 | 0.00 | — |
| LIO-SAM `map→base_link` | 0.76 | 0.00 | **0.01 m** |

导航闭环（`navigate_to_3d_pose` 目标 (1.5, 0)，起点 x=0.75）：

```
success=True error_code=0 total_time=4.69s distance=0.65m
```

## 5. 已知限制

| 项 | 现状 |
|---|---|
| 回环检测 | 已关（`loopClosureEnableFlag=false`），室内小场景够用；开回环需要更多 CPU |
| 地图 | 位姿在世界系（gz 原点），与 `maps/111` 静态图并不一致；做导航时建议 `map:=` 对应场景图，或后续接 `slam_toolbox`/保存 LIO-SAM 点位图 |
| 漂移 | 关回环后长时间运行会累积；短任务（<50 m）实测可用 |
| CPU | `numberOfCores=2`、`mappingProcessInterval=0.15`；点云 1800×16@10Hz |
| 起始位姿 | LIO-SAM 以启动位置为世界原点；gz 的 spawn 位姿若非 0，需要 `x_pose/y_pose` 与之一致 |

## 6. 相关文件

```
third_party/LIO-SAM/                     # 上游 ros2 分支 + 我们的 2 处 patch
src/pnc_nav_sim/params/lio_sam_sim3d.yaml
src/pnc_nav_sim/params/bridge_3d_lio.yaml
src/pnc_nav_sim/urdf/diff_drive_3d.urdf.xacro     # imu 传感器
src/pnc_nav_bringup/launch/gz_sim_3d_lidar_bringup.launch.py   # localization:=lio_sam
scripts/setup_lio_sam.sh
```
