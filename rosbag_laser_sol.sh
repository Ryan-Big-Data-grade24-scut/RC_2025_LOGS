#!/bin/bash

# 默认参数（当前时间）
CURRENT_YEAR=$(date +"%Y")
CURRENT_MONTH=$(date +"%m")
CURRENT_DAY=$(date +"%d")
CURRENT_HOUR=$(date +"%H")

# 显示帮助信息
usage() {
    echo "Usage: $0 -c CODE [-y YEAR] [-m MONTH] [-d DAY] [-h HOUR]"
    echo "Options:"
    echo "  -c, --code CODE    Experiment code (mandatory)"
    echo "  -y, --year YEAR    Year (default: $CURRENT_YEAR)"
    echo "  -m, --month MONTH  Month (default: $CURRENT_MONTH)"
    echo "  -d, --day DAY      Day (default: $CURRENT_DAY)"
    echo "  -h, --hour HOUR    Hour (default: $CURRENT_HOUR)"
    exit 1
}

# 参数解析
while [[ $# -gt 0 ]]; do
    case "$1" in
        -c|--code)
            CODE="$2"
            shift 2
            ;;
        -y|--year)
            YEAR="$2"
            shift 2
            ;;
        -m|--month)
            MONTH="$2"
            shift 2
            ;;
        -d|--day)
            DAY="$2"
            shift 2
            ;;
        -h|--hour)
            HOUR="$2"
            shift 2
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

# 设置默认值
YEAR="${YEAR:-$CURRENT_YEAR}"
MONTH="${MONTH:-$CURRENT_MONTH}"
DAY="${DAY:-$CURRENT_DAY}"
HOUR="${HOUR:-$CURRENT_HOUR}"

# 构建目录路径
BAG_DIR="./${YEAR}-${MONTH}-${DAY}_${HOUR}_${CODE}"
SOLUTION_BAG="$BAG_DIR/laser_solutions"

# 检查输入bag是否存在
INPUT_BAG="$BAG_DIR"/"laser_publisher"
if [ ! -d "$INPUT_BAG" ]; then
    echo "Error: Input bag directory not found: $INPUT_BAG"
    exit 1
fi

echo "=== Experiment Configuration ==="
echo "Timestamp: $YEAR-$MONTH-$DAY $HOUR:00"
echo "Experiment Code: $CODE"
echo "Input Bag: $INPUT_BAG"
echo "Output Bag: $SOLUTION_BAG"
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