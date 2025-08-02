#!/bin/bash

# 默认参数值
DEFAULT_CODE=""  # 默认实验代号
RECORD_TOPIC="/tf"        # 要录制的话题
RECORD_TOPIC2="/tf_static"        # 要录制的话题2
RECORD_TOPIC3="/laser_dist"      # 激光距离话题

# 要运行的节点命令
NODE_COMMAND1="ros2 launch localization_bringup localization_bringup.launch.py use_sim_time:=false lio:=pointlio use_relocalization:=false lidar_type:=mid360"
NODE_COMMAND2="ros2 run laser_rviz_demo laser_publisher"

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

echo "=== LIO Laser Experiment Setup ==="
echo "Node Command 1: $NODE_COMMAND1"
echo "Node Command 2: $NODE_COMMAND2"
echo "Record Topics: $RECORD_TOPIC, $RECORD_TOPIC2, $RECORD_TOPIC3"
echo "Output Directory: $OUTPUT_DIR"
echo "=================================="

# 启动第一个节点 (localization_bringup)
echo "Starting localization_bringup launch..."
$NODE_COMMAND1 &
LAUNCH_PID=$!

# 等待一下让第一个节点启动
sleep 3

# 启动第二个节点 (laser_publisher)
echo "Starting laser_publisher node..."
$NODE_COMMAND2 &
PUBLISHER_PID=$!

# 再等待一下让所有节点完全启动
sleep 2

# 开始录制数据
echo "Starting ros2 bag record for multiple topics..."
ros2 bag record -o "$OUTPUT_DIR/lio_laser_data" "$RECORD_TOPIC" "$RECORD_TOPIC2" "$RECORD_TOPIC3" &
RECORD_PID=$!

# 设置退出时的清理函数
cleanup() {
    echo "Shutting down..."
    
    # 终止所有进程
    if [ ! -z "$RECORD_PID" ]; then
        kill $RECORD_PID 2>/dev/null
    fi
    if [ ! -z "$PUBLISHER_PID" ]; then
        kill $PUBLISHER_PID 2>/dev/null
    fi
    if [ ! -z "$LAUNCH_PID" ]; then
        kill $LAUNCH_PID 2>/dev/null
    fi
    
    # 等待进程结束
    sleep 2
    
    echo "Experiment completed. Data saved to: $OUTPUT_DIR"
}
trap cleanup EXIT

# 等待用户按Ctrl+C退出
echo "All nodes started. Press Ctrl+C to stop recording and exit..."
echo "监控进程状态..."

# 监控主要进程是否还在运行
while true; do
    if ! kill -0 $LAUNCH_PID 2>/dev/null; then
        echo "Launch process has terminated unexpectedly!"
        break
    fi
    if ! kill -0 $PUBLISHER_PID 2>/dev/null; then
        echo "Publisher process has terminated unexpectedly!"
        break
    fi
    if ! kill -0 $RECORD_PID 2>/dev/null; then
        echo "Record process has terminated unexpectedly!"
        break
    fi
    sleep 5
done