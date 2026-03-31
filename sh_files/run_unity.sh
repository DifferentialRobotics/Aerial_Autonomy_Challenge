#!/bin/zsh
source devel/setup.zsh;
roslaunch ros_tcp_endpoint endpoint.launch & sleep 2;
src/AerialAutonomyChallenge-Simulator/AerialAutonomyChallenge-Simulator.x86_64 & sleep 3;
roslaunch diff_planner run_sim_unity.launch & sleep 3;
sh_files/sequential_waypoint_bridge.sh & sleep 2;
roslaunch diff_planner exp_rviz.launch;
wait;