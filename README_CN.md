# 2025空中机器人自主挑战赛 (Aerial Autonomy Challenge)  
# 2025 IROS COMPETITION


## 官方测试环境
> ros-noetic  
> ubuntu20.04  
> NVIDIA RTX4060
> INTEL I7

## 快速启动
#### 编译并启动Simulator-bridge
>+ `cd /path/to/AerialAutonomyChallenge-Simulator-bridge`  
>+ `catkin_make`
>+ `source devel/setup.bash`
>+ `roslaunch ros_tcp_endpoint endpoint.launch`

#### 启动Simulator
>+ `cd /path/to/AerialAutonomyChallenge-Simulator`  
>+ 双击`/path/to/AerialAutonomyChallenge-Simulator`下的`AerialAutonomyChallenge-Simulator.x86_64`

#### 编译并启动示例代码
>+ `cd /path/to/AerialAutonomyChallenge-Demo-EgoPlannerv2`  
>+ `catkin_make`
>+ `source devel/setup.bash`
>+ `roslaunch ego_planner single_drone_interactive.launch`

## 仿真器界面交互  
>+ W/A/S/D控制 镜头左右移动
>+ 按下鼠标左键控制摄像头PITCH，右键控制摄像头YAW，左右按键可同时按下
>+ 按下鼠标中键并拖动鼠标可实现W/A/S/D一样的移动效果
>+ 目前仿真器移动速度分别为：0.35m/s、0.35m/s、0.2m/s

## 仿真器ROS话题交互
#### 发布的话题：  
>+ /drone_0_pcl_render_node/cloud
#### 订阅的话题：  
>+ /quad_0/lidar_slam/odom

## 场景参数交互  
风场速调节：'[run_in_sim.xml](AerialAutonomyChallenge-Demo-EgoPlannerv2/src/planner/plan_manage/launch/include/run_in_sim.xml)'
#### 目前风速3级
>+ param name="wind_effect/strength" value="3.0"


## 致谢与声明

本项目部分功能包引用自 [MARSIM 仓库](https://github.com/hku-mars/MARSIM)，
由香港大学 MARS 团队开发，特此感谢其开源贡献。

相关代码严格遵循 MARSIM 的开源许可协议使用，用户在使用时请遵守原项目的许可证条款。


## Q&A

请随时提交问题或讨论,我们会在看到问题后尽快回复




