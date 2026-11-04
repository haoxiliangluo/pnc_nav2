Vendored from https://github.com/TixiaoShan/LIO-SAM (branch ros2)

pnc_nav2 改动（见 docs/guides/lio_sam_setup.md）：
1. src/mapOptmization.cpp：TF 改为 map -> base_link（参数 lidarToBase）
2. src/imuPreintegration.cpp：默认不发 IMU 预积分 TF（publishImuTf=false）
3. CMakeLists.txt：Eigen3 / GTSAM 用量固化到 /opt/gtsam_sys，RPATH

构建方式见 scripts/setup_lio_sam.sh
