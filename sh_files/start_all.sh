#!/bin/bash

# 获取当前脚本所在的工作空间根目录
WORKSPACE_ROOT=~/Diff-Planner-main
SRC_DIR=$WORKSPACE_ROOT/src

# 1. 初始化环境变量
source /opt/ros/noetic/setup.bash
source $WORKSPACE_ROOT/devel/setup.bash

echo "正在启动仿真环境..."

# 2. 启动 ROS-TCP-Endpoint (新窗口启动，避免阻塞)
# 注意：如果 package.xml 里的名字是大写，请将下方的 ros_tcp_endpoint 改为 ROS-TCP-Endpoint
gnome-terminal --title="ROS-TCP-Endpoint" -- bash -c "source $WORKSPACE_ROOT/devel/setup.bash; roslaunch ros_tcp_endpoint endpoint.launch; exec bash"

sleep 2 # 等待通信桥梁建立

# 3. 启动 Unity 执行文件
echo "正在启动 Unity 模拟器..."
chmod +x $SRC_DIR/AerialAutonomyChallenge-Simulator/AerialAutonomyChallenge-Simulator.x86_64
# 后台运行模拟器，防止阻塞脚本
$SRC_DIR/AerialAutonomyChallenge-Simulator/AerialAutonomyChallenge-Simulator.x86_64 & 

sleep 5 # 等待模拟器加载完毕

# 4. 启动路径规划节点 (新窗口启动)
gnome-terminal --title="Diff-Planner" -- bash -c "source $WORKSPACE_ROOT/devel/setup.bash; roslaunch diff_planner run_sim_unity.launch; exec bash"

sleep 2

# 5. 执行 sequential_waypoint_bridge 脚本
echo "正在执行航点桥接脚本..."
chmod +x $SRC_DIR/sequential_waypoint_bridge.sh
gnome-terminal --title="Waypoint-Bridge" -- bash -c "cd $SRC_DIR; ./sequential_waypoint_bridge.sh; exec bash"
roslaunch diff_planner exp_rviz.launch;
echo "所有组件已启动完成！"
