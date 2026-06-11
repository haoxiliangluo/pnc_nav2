#!/bin/bash
# 便捷启动脚本 - 自动切换到项目根目录并运行工具
#
# 用法:
#   ./run_tool.sh costmap_editor
#   ./run_tool.sh planner_tester
#   ./run_tool.sh nav_evaluator

set -e

# 查找项目根目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR/../.."

# 验证项目根目录
if [ ! -d "$PROJECT_ROOT/src/pnc_nav_bringup" ]; then
    echo "错误: 无法找到项目根目录"
    exit 1
fi

# 切换到项目根目录
cd "$PROJECT_ROOT"

# 解析工具名称
TOOL="$1"
shift  # 移除第一个参数，剩余参数传递给 Python 脚本

case "$TOOL" in
    costmap_editor|editor)
        echo "启动 costmap_editor..."
        python3 src/pnc_nav_utils/evaluation/costmap_editor.py "$@"
        ;;
    planner_tester|tester)
        echo "启动 planner_tester..."
        python3 src/pnc_nav_utils/evaluation/planner_tester.py "$@"
        ;;
    nav_evaluator|evaluator)
        echo "启动 nav_evaluator..."
        python3 src/pnc_nav_utils/evaluation/nav_evaluator.py "$@"
        ;;
    *)
        echo "用法: $0 <tool> [options]"
        echo ""
        echo "可用工具:"
        echo "  costmap_editor  (或 editor)    - 代价地图编辑器"
        echo "  planner_tester  (或 tester)    - C++ 规划器测试工具"
        echo "  nav_evaluator   (或 evaluator) - 导航性能评估工具"
        echo ""
        echo "示例:"
        echo "  $0 costmap_editor"
        echo "  $0 planner_tester --obstacles-yaml costmap_obstacles.yaml"
        echo "  $0 nav_evaluator"
        exit 1
        ;;
esac
