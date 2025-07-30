#!/bin/bash

# 固定参数（2025年7月25日的测试数据）
FIXED_DATE="2025_07_25"

# 显示帮助信息
usage() {
    echo "Usage: $0 -c CODE"
    echo "Options:"
    echo "  -c, --code CODE    Experiment code (mandatory)"
    exit 1
}

# 参数解析
while [[ $# -gt 0 ]]; do
    case "$1" in
        -c|--code)
            CODE="$2"
            shift 2
            ;;
        -h|--help)
            usage
            ;;
        *)
            echo "Unknown option: $1"
            usage
            ;;
    esac
done

# 验证必须参数
if [[ -z "$CODE" ]]; then
    echo "Error: --code parameter is mandatory!"
    usage
fi

# 构建路径
INPUT_BAG="rosbag2_${FIXED_DATE}-laser_${CODE}"
CURRENT_HOUR=$(date +"%H")
BAG_DIR="./$(date +"%Y-%m-%d")_${CURRENT_HOUR}_${CODE}"
SOLUTION_BAG="$BAG_DIR/laser_solutions"

# 检查输入bag是否存在
if [ ! -d "$INPUT_BAG" ]; then
    echo "Error: Input bag directory not found: $INPUT_BAG"
    echo "Hint: Expected format: rosbag2_${FIXED_DATE}-laser_<code>"
    exit 1
fi

# 创建输出目录
mkdir -p "$BAG_DIR" || { echo "Failed to create output directory"; exit 1; }

echo "=== Experiment Configuration ==="
echo "Input Bag: $INPUT_BAG (fixed date: $FIXED_DATE)"
echo "Output Directory: $BAG_DIR"
echo "Solution Bag: $SOLUTION_BAG"
echo "==============================="

# 执行命令链
ros2 bag play "$INPUT_BAG" &
PLAY_PID=$!

ros2 run laser_solver solver_node &
SOLVER_PID=$!

ros2 bag record /laser_solutions -o "$SOLUTION_BAG" &
RECORD_PID=$!

# 清理函数
cleanup() {
    echo "Shutting down processes..."
    kill -INT $PLAY_PID $SOLVER_PID $RECORD_PID 2>/dev/null
    wait $PLAY_PID $SOLVER_PID $RECORD_PID
    echo "Experiment completed. Solutions saved to: $SOLUTION_BAG"
}
trap cleanup EXIT

# 等待用户中断
echo "Press Ctrl+C to stop the experiment..."
wait $RECORD_PID