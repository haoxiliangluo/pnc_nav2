#!/usr/bin/env bash
# PNC Nav2 一键回归
#
#   ./scripts/run_all_tests.sh                # 构建 + 全部 gtest
#   ./scripts/run_all_tests.sh --sim          # 追加 2D 导航冒烟（成功/取消/不可达）
#   ./scripts/run_all_tests.sh --go2w         # 追加轮足 Go2-W 导航冒烟
#   ./scripts/run_all_tests.sh --terrain      # 追加坡道分段任务冒烟（地形 + DWA）
#   ./scripts/run_all_tests.sh --sim --go2w --terrain
#
# 退出码：0 全绿；1 有失败

set -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

RUN_SIM=0
RUN_GO2W=0
RUN_TERRAIN=0
for arg in "$@"; do
  case "$arg" in
    --sim) RUN_SIM=1 ;;
    --go2w) RUN_GO2W=1 ;;
    --terrain) RUN_TERRAIN=1 ;;
    -h|--help) grep '^#' "$0" | head -12; exit 0 ;;
  esac
done

PACKAGES="pnc_nav_interfaces pnc_nav_core pnc_nav_planners pnc_nav_map \
pnc_nav_task pnc_nav_locomotion pnc_nav_diagnostics pnc_nav_sim pnc_nav_bringup pnc_nav_examples"

export ROS_DOMAIN_ID="${PNC_TEST_DOMAIN_ID:-43}"
export GZ_PARTITION="${PNC_TEST_GZ_PARTITION:-pnc_nav_test}"

source /opt/ros/jazzy/setup.bash
export PATH="/opt/ros/jazzy/opt/gz_tools_vendor/bin:${PATH}"

FAIL=0

cleanup_procs() {
  python3 - <<'PY'
import os, signal
me = os.getpid()
def ancestors(p):
    out = set()
    while p and p != 1:
        out.add(p)
        try:
            p = int(open(f'/proc/{p}/stat').read().split()[3])
        except Exception:
            break
    return out
skip = ancestors(me)
for pid in os.listdir('/proc'):
    if not pid.isdigit():
        continue
    pid = int(pid)
    if pid in skip:
        continue
    try:
        cmd = open(f'/proc/{pid}/cmdline', 'rb').read().decode(errors='ignore').replace('\0', ' ')
    except Exception:
        continue
    if 'pnc_nav2/install' in cmd and 'grep' not in cmd:
        try:
            os.kill(pid, signal.SIGKILL)
        except Exception:
            pass
PY
  sleep 1
}

wait_ready() {
  # 就绪判据：action server 出现 + /odom、/map 话题存在（不再依赖 tf2_echo 的启动耗时）
  local base_frame="${1:-base_link}"
  for _ in $(seq 1 90); do
    if timeout 6 ros2 action list 2>/dev/null | grep -q navigate_to_3d_pose; then
      if timeout 6 ros2 topic list 2>/dev/null | grep -qx /odom; then
        if timeout 6 ros2 topic list 2>/dev/null | grep -qx /map; then
          sleep 4      # 给 TF/建图稳定一点时间
          return 0
        fi
      fi
    fi
    sleep 1
  done
  echo "  (等待超时：base_frame=${base_frame})"
  return 1
}

echo "=== 1/3 构建（增量）==="
if ! colcon build --symlink-install --packages-select ${PACKAGES} --parallel-workers 4 > /tmp/pnc_build.log 2>&1; then
  echo "构建失败，日志：/tmp/pnc_build.log"
  tail -20 /tmp/pnc_build.log
  exit 1
fi
source install/setup.bash
# 干净重启 ros2 daemon：长期运行的 daemon 会缓存旧图，导致 action list 卡住/过期
ros2 daemon stop > /dev/null 2>&1 || true
ros2 daemon start > /dev/null 2>&1 || true
echo "构建 OK"

echo ""
echo "=== 2/3 单元测试（gtest）==="
colcon test --packages-select ${PACKAGES} > /tmp/pnc_test.log 2>&1
python3 - <<'PY'
import glob, re, sys
files = glob.glob('build/**/test_results/**/*.gtest.xml', recursive=True)
total = failures = 0
per_pkg = {}
for f in files:
    try:
        text = open(f, encoding='utf-8', errors='ignore').read()
    except OSError:
        continue
    m = re.search(r'tests="(\d+)".*?failures="(\d+)"', text)
    if not m:
        continue
    t, fl = int(m.group(1)), int(m.group(2))
    total += t
    failures += fl
    pkg = f.split('/pnc_nav_')[1].split('/')[0] if '/pnc_nav_' in f else 'other'
    per_pkg.setdefault(pkg, [0, 0])
    per_pkg[pkg][0] += t
    per_pkg[pkg][1] += fl
for pkg, (t, fl) in sorted(per_pkg.items()):
    flag = 'OK ' if fl == 0 else 'FAIL'
    print(f'  [{flag}] {pkg:10s} {t:3d} 例 / {fl} 失败')
print(f'合计 {total} 例，失败 {failures} 例')
sys.exit(0 if failures == 0 else 1)
PY
[ $? -ne 0 ] && FAIL=1

echo ""
echo "=== 3/3 场景冒烟 ==="

if [ ${RUN_SIM} -eq 1 ]; then
  echo "--- [2D] profile:=sim2d ---"
  cleanup_procs
  ros2 launch pnc_nav_bringup bringup.launch.py profile:=sim2d use_gui:=false use_rviz:=false \
    > /tmp/pnc_sim_bringup.log 2>&1 &
  if wait_ready; then
    for case in feedback cancel unreachable; do
      echo "  -> 用例 ${case}"
      if timeout 90 python3 test/action_smoke_test.py "${case}" | tail -2; then
        echo "     OK"
      else
        echo "     FAIL"
        FAIL=1
      fi
    done
  else
    echo "  启动超时，见 /tmp/pnc_sim_bringup.log"; FAIL=1
  fi
  cleanup_procs
fi

if [ ${RUN_GO2W} -eq 1 ]; then
  echo "--- [轮足] profile:=sim_go2w ---"
  cleanup_procs
  ros2 launch pnc_nav_bringup bringup.launch.py profile:=sim_go2w use_gui:=false use_rviz:=false \
    > /tmp/pnc_go2w_bringup.log 2>&1 &
  if wait_ready base; then
    if timeout 120 python3 test/action_smoke_test.py feedback 0.6 0.0 | tail -2; then
      echo "     OK"
    else
      echo "     FAIL"
      FAIL=1
    fi
  else
    echo "  启动超时，见 /tmp/pnc_go2w_bringup.log"; FAIL=1
  fi
  cleanup_procs
fi

if [ ${RUN_TERRAIN} -eq 1 ]; then
  echo "--- [坡道] slope_test + 地形 + 分段任务 ---"
  cleanup_procs
  ros2 launch pnc_nav_bringup gz_sim_3d_lidar_bringup.launch.py \
    use_gui:=false use_rviz:=false use_elevation:=true world:=slope_test.sdf \
    params_file:="$ROOT/install/pnc_nav_bringup/share/pnc_nav_bringup/config/nav_params_terrain.yaml" \
    > /tmp/pnc_terrain_bringup.log 2>&1 &
  if wait_ready; then
    ros2 launch pnc_nav_task mission.launch.py \
      params_file:="$ROOT/install/pnc_nav_task/share/pnc_nav_task/config/mission_slope.yaml" \
      auto_start:=true > /tmp/pnc_terrain_mission.log 2>&1 &
    if timeout 300 python3 test/mission_smoke_test.py 280 | tail -4; then
      echo "     OK"
    else
      echo "     FAIL"
      FAIL=1
    fi
  else
    echo "  启动超时，见 /tmp/pnc_terrain_bringup.log"; FAIL=1
  fi
  cleanup_procs
fi

echo ""
if [ ${FAIL} -eq 0 ]; then
  echo "=== 全部通过 ==="
else
  echo "=== 有失败项，见上面输出 / /tmp/pnc_*.log ==="
fi
exit ${FAIL}
