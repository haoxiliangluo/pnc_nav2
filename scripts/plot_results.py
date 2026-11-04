#!/usr/bin/env python3
"""把 experiment_runner 的 summary.csv 画成图（论文/报告可直接用）。

用法：
  python3 scripts/plot_results.py experiments/results/<目录>
输出：
  <目录>/plot_success_rate.png
  <目录>/plot_duration.png
  <目录>/plot_distance.png
"""
import csv
import os
import sys
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def load(path):
    rows = list(csv.DictReader(open(path, newline='')))
    data = defaultdict(list)
    for r in rows:
        data[r['case']].append(r)
    return data


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    out_dir = sys.argv[1]
    data = load(os.path.join(out_dir, 'summary.csv'))
    cases = list(data.keys())

    # 1) 成功率
    rates = [sum(1 for r in data[c] if r['success'] == 'True') / len(data[c]) for c in cases]
    plt.figure(figsize=(6, 3.2))
    plt.bar(cases, [r * 100 for r in rates], color='#4c72b0')
    plt.ylabel('success rate (%)')
    plt.ylim(0, 110)
    for i, v in enumerate(rates):
        plt.text(i, v * 100 + 2, f'{v * 100:.0f}%', ha='center')
    plt.title('Success rate')
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'plot_success_rate.png'), dpi=150)

    # 2) 耗时
    plt.figure(figsize=(6, 3.2))
    times = [[float(r['total_time']) for r in data[c] if r['success'] == 'True'] or [0.0]
             for c in cases]
    plt.bar(cases, [sum(t) / len(t) for t in times], color='#55a868')
    for i, t in enumerate(times):
        plt.scatter([i] * len(t), t, color='#222', s=12, zorder=3)
    plt.ylabel('total time (s)')
    plt.title('Navigation duration (successful runs)')
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'plot_duration.png'), dpi=150)

    # 3) 里程
    plt.figure(figsize=(6, 3.2))
    dists = [[float(r['distance_traveled']) for r in data[c] if r['success'] == 'True'] or [0.0]
             for c in cases]
    plt.bar(cases, [sum(d) / len(d) for d in dists], color='#c44e52')
    for i, d in enumerate(dists):
        plt.scatter([i] * len(d), d, color='#222', s=12, zorder=3)
    plt.ylabel('distance traveled (m)')
    plt.title('Traveled distance (successful runs)')
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'plot_distance.png'), dpi=150)

    print(f"图已生成：{out_dir}/plot_*.png")
    return 0


if __name__ == '__main__':
    sys.exit(main())
