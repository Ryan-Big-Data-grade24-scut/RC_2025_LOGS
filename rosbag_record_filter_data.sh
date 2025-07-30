#!/bin/bash

# 默认参数值
DEFAULT_CODE=""  # 默认实验代号
RECORD_TOPICS="/laser_dist /laser_solutions /tf /tf_static"        # 要录制的话题
# 要运行的节点命令
NODE_COMMANDS=(
    "ros2 launch livox_ros_driver2 msg_MID360_upsideDown.launch.py"
    "ros2 launch localization_bringup localization_bringup.launch.py use_sim_time:=false lio:=pointlio use_relocalization:=false lidar_type:=mid360"
    "ros2 launch laser_solution_filter laser_test.launch.py"
)

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
echo "Node Commands:"
for cmd in "${NODE_COMMANDS[@]}"; do
    echo "  - $cmd"
done
echo "Record Topics: $RECORD_TOPICS"
echo "Output Directory: $OUTPUT_DIR"
echo "========================="

# 启动ROS2节点
echo "Starting ROS2 nodes in background..."
NODE_PIDS=()

for cmd in "${NODE_COMMANDS[@]}"; do
    echo "Starting: $cmd"
    $cmd &
    NODE_PIDS+=($!)
done


# 检查节点是否运行
failed_nodes=0
for i in "${!NODE_PIDS[@]}"; do
    if ! ps -p ${NODE_PIDS[$i]} > /dev/null; then
        echo "Error: Failed to start node: ${NODE_COMMANDS[$i]}"
        failed_nodes=$((failed_nodes + 1))
    fi
done

if [ $failed_nodes -gt 0 ]; then
    echo "Warning: $failed_nodes node(s) failed to start, but continuing..."
fi

# 开始录制数据
echo "Starting ros2 bag record..."
ros2 bag record -o "$OUTPUT_DIR/experiment_data" $RECORD_TOPICS &
RECORD_PID=$!

# 设置退出时的清理函数
cleanup() {
    echo "Shutting down..."
    # 停止录制
    if [ ! -z "$RECORD_PID" ]; then
        kill -INT $RECORD_PID 2>/dev/null
    fi
    # 停止所有节点
    for pid in "${NODE_PIDS[@]}"; do
        kill -INT $pid 2>/dev/null
    done
    # 等待进程结束
    if [ ! -z "$RECORD_PID" ]; then
        wait $RECORD_PID 2>/dev/null
    fi
    for pid in "${NODE_PIDS[@]}"; do
        wait $pid 2>/dev/null
    done
    echo "Experiment completed. Data saved to: $OUTPUT_DIR"
}
trap cleanup EXIT

# 等待用户按Ctrl+C退出
echo "Press Ctrl+C to stop recording and exit..."
wait $RECORD_PID