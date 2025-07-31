#!/bin/bash

# 默认参数值
DEFAULT_CODE=""  # 默认实验代号
RECORD_TOPIC="/tf"        # 要录制的话题
RECORD_TOPIC2="/tf_static"        # 要录制的话题2

NODE_COMMAND=""  # 要运行的节点

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


# 开始录制数据
echo "Starting ros2 bag record..."
ros2 bag record -o "$OUTPUT_DIR/tf" "$RECORD_TOPIC" &
RECORD_PID=$!
ros2 bag record -o "$OUTPUT_DIR/tf_static" "$RECORD_TOPIC" &
RECORD_PID=$!

# 设置退出时的清理函数
cleanup() {
    echo "Shutting down..."
    echo "Experiment completed. Data saved to: $OUTPUT_DIR"
}
trap cleanup EXIT

# 等待用户按Ctrl+C退出
echo "Press Ctrl+C to stop recording and exit..."
wait $RECORD_PID