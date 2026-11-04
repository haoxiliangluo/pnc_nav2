# 实验目录（Experiment Harness）

用于跑"可复现、可对比"的导航实验，产出 CSV + 图，供论文/报告使用。

## 目录约定

```
experiments/
├── <实验名>.yaml          # 实验配置（profile / goal / runs / cases）
├── params/                # 实验专用参数文件（如 DWA 配置）
└── results/<时间戳>_<实验名>/
    ├── runs.json          # 每次运行的完整结果
    ├── summary.csv        # 汇总表（case/run/success/error_code/time/distance）
    └── plot_*.png         # 成功率 / 耗时 / 里程图
```

## 跑一个实验

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=43 GZ_PARTITION=pnc_nav_test

python3 scripts/experiment_runner.py experiments/planner_comparison.yaml
python3 scripts/plot_results.py experiments/results/<目录>
```

> 出图需要 matplotlib；本机 `~/.local` 里的 numpy 2.x 与系统 matplotlib 不兼容，
> 用 `PYTHONNOUSERSITE=1 python3 scripts/plot_results.py ...` 走系统 numpy 1.26。

## 现有实验

| 配置文件 | 内容 |
|---|---|
| `planner_comparison.yaml` | AStar2D+PurePursuit vs AStar2D+DWA（各 3 次） |
| `planner_extended.yaml` | 加 RRT*、直线规划示例插件（各 2 次） |
| `costmap_ablation.yaml` | 静态图 vs 静态+障碍层（前方 spawn 0.4m 箱子，各 3 次） |
| `cross_platform.yaml` | 2D 差速车 vs 轮足 Go2-W（同一目标，各 2 次） |

> case 级字段：`profile`（覆盖全局）、`params_file`、`setup_cmd` + `setup_model_name`（spawn 障碍并校验）。

## 配置字段

```yaml
name: 实验名
profile: sim2d            # bringup profile
goal: [2.0, 0.0]          # 目标点
runs: 3                   # 每个 case 重复次数
timeout: 120              # 单次导航超时 (s)
cases:
  - name: 用例名
    params_file: experiments/params/xxx.yaml   # 留空 = 默认参数
```
