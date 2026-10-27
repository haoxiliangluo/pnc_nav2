# WP3 测试与回归（测试 agent）

> 负责人：测试 agent ｜ 状态：已实现（run_all_tests.sh + docs/testing.md）
> 依赖：WP1/WP2 的接口；现有 26 个 gtest + 3 个冒烟脚本

## 目标
一条命令跑完"单测 + 仿真冒烟 + 结果汇总"，让每次改动有统一回归口径。

## 交付物
- `scripts/run_all_tests.sh`：
  - `colcon build`（增量）→ `colcon test`（全部 gtest）→ 汇总
  - `--sim` 模式：起 `bringup profile:=sim2d` → 跑三个冒烟用例（成功/取消/不可达）→ 关仿真
  - 退出码：任一失败非 0；输出简明摘要表
- `docs/testing.md`：测试分层（L1 单测 / L2 节点集成 / L3 仿真冒烟）、怎么加用例、已知不稳定项
- 汇总当前基线（写进文档）：
  - gtest：`pnc_nav_map` 17、`pnc_nav_task` 7、`pnc_nav_planners` 2
  - 冒烟：可达 `success/code=0`；取消 `code=4`；不可达 `code=3`

## 验收
1. `./scripts/run_all_tests.sh` 在干净终端（不 source）也能跑通
2. `./scripts/run_all_tests.sh --sim` 全绿（无图形环境，默认 `use_gui:=false`）
3. 文档写清"哪一层测什么、失败怎么定位"

## 不做
- CI 服务器接入、覆盖率门禁、Gazebo 物理级回归
