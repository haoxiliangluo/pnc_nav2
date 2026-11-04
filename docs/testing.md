# 测试与回归（Testing）

> 最后核对：commit `7eb2a59` + 本轮（2026-10-23）
> 一条命令：`./scripts/run_all_tests.sh`（加 `--sim` 跑仿真冒烟）

---

## 1. 三层测试

| 层 | 内容 | 命令 | 速度 |
|---|---|---|---|
| **L1 单元** | 纯函数/状态机（gtest，不依赖仿真） | `colcon test --packages-select <pkg>` | 秒级 |
| **L2 节点冒烟** | action 生命周期（成功/取消/不可达），**断言 error_code** | `python3 test/action_smoke_test.py <case>` | 数十秒（需仿真） |
| **L3 仿真回归** | 起 `bringup profile:=sim2d` 跑 L2 三用例 | `./scripts/run_all_tests.sh --sim` | 2–3 分钟 |

> 原则：**L1 抓算法与策略，L2 抓接口契约，L3 抓集成**。L3 数量要少（慢且受启动时序影响）。

---

## 2. 当前基线

### L1（30 例，全绿）
| 包 | 用例数 | 覆盖 |
|---|---|---|
| `pnc_nav_map` | 16 | 坡度/台阶/粗糙度、可通行性→代价、高程网格、补洞、噪声回归 |
| `pnc_nav_task` | 7 | 顺序推进、循环、重试、暂停/恢复/取消 |
| `pnc_nav_locomotion` | 5 | 零速中性、幅值随速度、关节运动、急停、频率 |
| `pnc_nav_planners` | 2 | PP2D 正向前进、路径在身后必须原地转（KI-020 回归） |

### L2/L3（冒烟）
| 用例 | 期望 | 实测基线 |
|---|---|---|
| `feedback` | `success=true, error_code=0` | 可达 2m：~10–16s / 1.9m |
| `cancel` | `error_code=4` | 2s 后取消，车停 |
| `unreachable` | `error_code=3` | 3 次恢复后失败，~3–13s |

冒烟脚本**会校验** `error_code`，不匹配则非 0 退出（避免"跑完就算过"）。

---

## 2.5 实验（论文/报告用数据）

```bash
python3 scripts/experiment_runner.py experiments/planner_comparison.yaml   # 跑实验
PYTHONNOUSERSITE=1 python3 scripts/plot_results.py experiments/results/<目录>  # 出图
```

- 配置在 `experiments/*.yaml`（profile / goal / runs / cases），结果落
  `experiments/results/<时间戳>_<实验名>/`（runs.json + summary.csv + plot_*.png）
- 首组基线（2D 平地、目标 (2,0)、各 3 次，全部成功）：

| 用例 | 耗时 (s) | 里程 (m) |
|---|---|---|
| AStar2D + PurePursuit2D | 9.09 / 13.50 / 13.56 | 1.90 / 1.91 / 1.91 |
| AStar2D + DWA2D | 9.44 / 9.45 / 9.49 | 1.86 / 1.86 / 1.86 |

> DWA 更稳定（耗时方差小）；PP 有一次更快（9.09s）但波动大。
> 注意：出图要用 `PYTHONNOUSERSITE=1`（`~/.local` 的 numpy 2.x 与系统 matplotlib 不兼容）。

## 3. 怎么加用例

- **算法/策略** → 加 gtest（参考 `pnc_nav_map/test/` 与 `pnc_nav_task/test/`）
- **接口契约**（action/服务/话题）→ 加 `action_smoke_test.py` 的 case 或新脚本，必须断言结果
- **仿真场景** → 优先写进 world SDF（运行期 spawn 的模型对 3D 激光不可见，见 KI-016）

## 4. 失败定位

| 症状 | 先看 |
|---|---|
| 构建失败 | `/tmp/pnc_build.log` |
| gtest 失败 | `build/**/test_results/**/*.gtest.xml`、`/tmp/pnc_test.log` |
| 仿真冒烟失败 | `/tmp/pnc_sim_bringup.log`、`/tmp/pnc_test.log` |
| 第一个用例报 `TF_FAILURE` | 启动时序：脚本已等待 `map→base_link` 就绪；仍失败查 `ros2 run tf2_ros tf2_echo map base_link` |

## 5. 已知不稳定项

| 项 | 说明 | 缓解 |
|---|---|---|
| 仿真启动时序 | action server 比 TF 先就绪 | 脚本等待 TF 后再跑用例 |
| 残留进程 | 上一个 launch 的 gz 没退干净会导致 TF 两棵树 | 脚本启动前会清 `pnc_nav2/install` 进程 |
| 运行期 spawn 模型 | 3D 激光看不到（KI-016） | 用 world SDF 里的静态障碍 |
| 测试用 domain/partition | 默认 `ROS_DOMAIN_ID=43`、`GZ_PARTITION=pnc_nav_test` | 环境变量可覆盖 |

## 6. 变更记录

| 日期 | 变更 |
|---|---|
| 2026-10-23 | 初版：三层测试、30 例基线、一键脚本与失败定位 |
