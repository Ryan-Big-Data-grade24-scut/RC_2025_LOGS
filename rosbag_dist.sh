#!/bin/bash

# 默认参数值
DEFAULT_CODE=""  # 默认实验代号
RECORD_TOPIC="/laser_dist"        # 要录制的话题
NODE_COMMAND="ros2 launch laser_solver flash_vis.launch.py"  # 要运行的节点

# 显示帮助信息
usage() {
    echo "Usage: $0 [-c CODE] [-h]"
    echo "Options:"
    echo "  -c, --code CODE    Specify experiment code (required, no default)"
    echo "  -h, --help         Show this help message"
    exit 0
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
            exit 1
            ;;
    esac
done

# 设置默认实验代号
#CODE="${CODE:-$DEFAULT_CODE}"

if [[ -z "$CODE" ]]; then
    echo "Error: --code parameter is mandatory!"
    usage
    exit 1
fi

# 创建带时间戳和代号的目录
TIMESTAMP=$(date +"%Y-%m-%d_%H")  # 精确到小时的时间戳
OUTPUT_DIR="./${TIMESTAMP}_${CODE}  "
mkdir -p "$OUTPUT_DIR"

if [ $? -ne 0 ]; then
    echo "Error: Failed to create output directory $OUTPUT_DIR"
    exit 1
fi

echo "=== Experiment Setup ==="
echo "Node Command: $NODE_COMMAND"
echo "Record Topic: $RECORD_TOPIC"
echo "Output Directory: $OUTPUT_DIR"
echo "========================="

# 启动ROS2节点
echo "Starting ROS2 node in background..."
$NODE_COMMAND &
NODE_PID=$!

# 等待节点初始化
sleep 2

# 检查节点是否运行
if ! ps -p $NODE_PID > /dev/null; then
    echo "Error: Failed to start ROS2 node!"
    exit 1
fi

# 开始录制数据
echo "Starting ros2 bag record..."
ros2 bag record -o "$OUTPUT_DIR/laser_publisher" "$RECORD_TOPIC" &
RECORD_PID=$!

# 设置退出时的清理函数
cleanup() {
    echo "Shutting down..."
    kill -INT $NODE_PID $RECORD_PID 2>/dev/null
    wait $NODE_PID $RECORD_PID
    echo "Experiment completed. Data saved to: $OUTPUT_DIR"
}
trap cleanup EXIT

# 等待用户按Ctrl+C退出
echo "Press Ctrl+C to stop recording and exit..."
wait $RECORD_PID