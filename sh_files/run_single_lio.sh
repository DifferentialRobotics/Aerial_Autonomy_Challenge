#!/bin/zsh
echo 'nv' | sudo -S chmod 777 /dev/tty* & sleep 1;
export DRONE_ID=0;
roslaunch mavros px4.launch & sleep 2;
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 & sleep 1;   # ATTITUDE_QUATERNION
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 & sleep 1;  # HIGHRES_IMU
rosrun mavros mavcmd long 511 83 5000 0 0 0 0 0 & sleep 1;   # ATTITUDE_TARGET
rosrun mavros mavcmd long 511 147 5000 0 0 0 0 0 & sleep 1;  # BATTERY_STATUS
rosrun mavros mavcmd long 511 106 5000 0 0 0 0 0 & sleep 1;  
source devel/setup.zsh;
roslaunch faster_lio mapping_mid360.launch & sleep 10;
roslaunch ekf ekf_lidar.launch & sleep 5;
roslaunch diff_planner run_exp_single_lio.launch & sleep 3;
roslaunch px4ctrl run_ctrl_lio.launch & sleep 3;
roslaunch multipoint multipointplan_exp_lio.launch & sleep 2;
roslaunch diff_planner exp_rviz.launch & sleep 1;
wait;
