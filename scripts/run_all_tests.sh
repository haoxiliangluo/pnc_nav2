#!/usr/bin/env bash
# PNC Nav2 一键回归：构建 + 单测（+ 可选仿真冒烟）
#
#   ./scripts/run_all_tests.sh          # 构建 + 全部 gtest
#   ./scripts/run_all_tests.sh --sim    # 额外跑仿真冒烟（可达/取消/不可达）
#
# 退出码：0 全绿；1 有失败

set -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

MODE="${1:-}"
PACKAGES="pnc_nav_interfaces pnc_nav_core pnc_nav_planners pnc_nav_map \
pnc_nav_task pnc_nav_locomotion pnc_nav_diagnostics pnc_nav_sim pnc_nav_bringup pnc_nav_examples"

export ROS_DOMAIN_ID="${PNC_TEST_DOMAIN_ID:-43}"
export GZ_PARTITION="${PNC_TEST_GZ_PARTITION:-pnc_nav_test}"

source /opt/ros/jazzy/setup.bash
export PATH="/opt/ros/jazzy/opt/gz_tools_vendor/bin:${PATH}"

FAIL=0
echo "=== 1/3 构建（增量）==="
if ! colcon build --symlink-install --packages-select ${PACKAGES} --parallel-workers 4 > /tmp/pnc_build.log 2>&1; then
  echo "构建失败，日志：/tmp/pnc_build.log"
  tail -20 /tmp/pnc_build.log
  exit 1
fi
source install/setup.bash
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
GTEST_RC=$?
[ $GTEST_RC -ne 0 ] && FAIL=1

if [ "$MODE" = "--sim" ]; then
  echo ""
  echo "=== 3/3 仿真冒烟（profile:=sim2d）==="

  # 清掉可能残留的本项目进程（只杀 pnc_nav2 install 路径下的）
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

  ros2 launch pnc_nav_bringup bringup.launch.py profile:=sim2d use_gui:=false use_rviz:=false \
    > /tmp/pnc_sim_bringup.log 2>&1 &
  LAUNCH_PID=$!
  trap 'kill -INT ${LAUNCH_PID} 2>/dev/null' EXIT

  # 等 action server 起来（最多 60s）
  for _ in $(seq 1 60); do
    if ros2 action list 2>/dev/null | grep -q navigate_to_3d_pose; then break; fi
    sleep 1
  done

  # 再等 TF 与地图就绪（冒烟第一步依赖 map->base_link，否则会立刻 TF_FAILURE）
  for _ in $(seq 1 60); do
    if timeout 2 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null | grep -q Translation; then
      break
    fi
    sleep 1
  done
  sleep 2

  for case in feedback cancel unreachable; do
    echo "  -> 用例 ${case}"
    if timeout 90 python3 test/action_smoke_test.py "${case}" | tail -3; then
      echo "     OK"
    else
      echo "     FAIL"
      FAIL=1
    fi
  done
fi

echo ""
if [ ${FAIL} -eq 0 ]; then
  echo "=== 全部通过 ==="
else
  echo "=== 有失败项，见上面输出 / /tmp/pnc_test.log ==="
fi
exit ${FAIL}
