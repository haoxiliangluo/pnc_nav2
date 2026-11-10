#!/usr/bin/env python3
"""清理 pnc_nav 仿真进程（按进程组整组杀，避免 pkill -f 匹配到自身）。"""
import os
import signal
import subprocess

KEYS = ('pnc_nav_bringup', 'pnc_nav_sim/install', 'install/lio_sam',
        'pnc_nav_core', 'pnc_nav_map/', 'pnc_nav_task', 'points_time_adder',
        'gz_sim_3d_lidar', 'mission.launch', 'elevation_map_node')

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
out = subprocess.run(['ps', '-eo', 'pid,pgid,args'], capture_output=True, text=True).stdout.splitlines()
groups, pids = set(), set()
for line in out[1:]:
    parts = line.split(None, 2)
    if len(parts) < 3:
        continue
    pid, pgid, cmd = int(parts[0]), int(parts[1]), parts[2]
    if pid in skip or pid == me:
        continue
    if any(k in cmd for k in KEYS):
        groups.add(pgid)
        pids.add(pid)
for g in groups:
    try:
        os.killpg(g, signal.SIGKILL)
    except Exception:
        pass
for p in pids:
    try:
        os.kill(p, signal.SIGKILL)
    except Exception:
        pass
print(f'killed groups={len(groups)} pids={len(pids)}')
