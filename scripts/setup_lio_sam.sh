#!/usr/bin/env bash
# 一键准备 LIO-SAM 依赖（Ubuntu 24.04 / ROS 2 Jazzy 实测）
#
# 背景：apt 的 libgtsam-dev 带的是 4.2.0，但
#   1) 打包缺 CppUnitLite.a（CMake 直接报错）
#   2) 没有 gtsam_unstable（LIO-SAM 的 imuPreintegration 需要 IncrementalFixedLagSmoother）
#   3) 上游 GTSAM 若用自带 Eigen，会和系统 Eigen 3.4.0 冲突（static_assert）
# 所以这里：apt 装基础依赖 + 源码编 GTSAM（GTSAM_USE_SYSTEM_EIGEN=ON, BUILD_UNSTABLE=ON）到 /opt/gtsam_sys，
# 并暂时禁用 apt 的 GTSAM cmake config，避免 CMake 抢匹配。
#
# 用法：./scripts/setup_lio_sam.sh
set -euo pipefail

GTSAM_VERSION="4.2.0"
GTSAM_PREFIX="/opt/gtsam_sys"
SRC_DIR="${HOME}/.cache/pnc_nav2/gtsam"

echo "==> [1/5] 安装依赖"
sudo apt-get install -y libgtsam-dev libmetis-dev libboost-all-dev libtbb-dev libeigen3-dev

echo "==> [2/5] 修补 apt 包缺失的 CppUnitLite.a（LIO-SAM 不用它，建空占位库即可）"
if [ ! -f /usr/lib/x86_64-linux-gnu/libCppUnitLite.a ]; then
  sudo ar rcs /usr/lib/x86_64-linux-gnu/libCppUnitLite.a
fi

echo "==> [3/5] 源码编译 GTSAM ${GTSAM_VERSION}（含 gtsam_unstable，用系统 Eigen）"
if [ ! -f "${GTSAM_PREFIX}/include/gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h" ]; then
  mkdir -p "$(dirname "${SRC_DIR}")"
  if [ ! -d "${SRC_DIR}" ]; then
    # 直连 GitHub 不通时走镜像
    git -c http.proxy= clone --depth 1 -b "${GTSAM_VERSION}" \
      https://gh-proxy.com/https://github.com/borglab/gtsam.git "${SRC_DIR}"
  fi
  cmake -S "${SRC_DIR}" -B "${SRC_DIR}/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_BUILD_UNSTABLE=ON \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_PYTHON=OFF \
    -DGTSAM_WITH_TBB=OFF \
    -DCMAKE_INSTALL_PREFIX="${GTSAM_PREFIX}" > /tmp/gtsam_cmake.log 2>&1
  cmake --build "${SRC_DIR}/build" -j"$(nproc --ignore=1)" > /tmp/gtsam_build.log 2>&1
  sudo cmake --install "${SRC_DIR}/build" > /tmp/gtsam_install.log 2>&1
else
  echo "    已存在，跳过"
fi

echo "==> [4/5] 禁用 apt 的 GTSAM cmake config（让 CMake 找到 ${GTSAM_PREFIX}）"
for d in /usr/lib/cmake/GTSAM /usr/lib/cmake/GTSAM_UNSTABLE; do
  if [ -d "$d" ]; then sudo mv "$d" "$d.disabled"; fi
done

echo "==> [5/5] LIO-SAM 源码（ros2 分支，已含 pnc_nav 的 TF/外参改造）"
echo "    仓库内路径：third_party/LIO-SAM"
echo
echo "编译 LIO-SAM（关键：Eigen 已一致，只需指向 /opt/gtsam_sys）："
echo "  source /opt/ros/jazzy/setup.bash && source install/setup.bash"
echo "  export CMAKE_PREFIX_PATH=${GTSAM_PREFIX}:\$CMAKE_PREFIX_PATH"
echo "  export LD_LIBRARY_PATH=${GTSAM_PREFIX}/lib:\$LD_LIBRARY_PATH"
echo "  colcon build --packages-select lio_sam"
echo
echo "运行（LIO-SAM 3D 定位）："
echo "  ros2 launch pnc_nav_bringup gz_sim_3d_lidar_bringup.launch.py localization:=lio_sam"
echo "完成。"
