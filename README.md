# 2025 Aerial Autonomy Challenge 
# 2025 IROS COMPETITION

## Official Test Environment
> ros-noetic  
> ubuntu20.04  
> NVIDIA RTX4060
> INTEL I7

## Quick Start
#### Build and Launch Simulator-bridge
>+ `cd /path/to/AerialAutonomyChallenge-Simulator-bridge`  
>+ `catkin_make`
>+ `source devel/setup.bash`
>+ `roslaunch ros_tcp_endpoint endpoint.launch`

#### Launch Simulator
>+ `cd /path/to/AerialAutonomyChallenge-Simulator`  
>+ Double-click`AerialAutonomyChallenge-Simulator.x86_64`inside the simulator folder

#### Build and Launch Demo Code
>+ `cd /path/to/AerialAutonomyChallenge-Demo-EgoPlannerv2`  
>+ `catkin_make`
>+ `source devel/setup.bash`
>+ `roslaunch ego_planner single_drone_interactive.launch`

## UI Interaction
>+ Use W/A/S/D to move the camera horizontally
>+ Left mouse button controls camera pitch, right button controls yaw; both buttons can be held together
>+ Hold and drag the middle mouse button to move the camera similarly to W/A/S/D
>+ Current movement speeds in simulator: 0.35 m/s, 0.35 m/s, 0.2 m/s

## ROS Topic Interaction
#### Published Topics:  
>+ /drone_0_pcl_render_node/cloud
#### Subscribed Topics:  
>+ /quad_0/lidar_slam/odom

## Scenario Parameter Configuration 
Wind field strength is configurable in：'[run_in_sim.xml](AerialAutonomyChallenge-Demo-EgoPlannerv2/src/planner/plan_manage/launch/include/run_in_sim.xml)'
#### Current Wind Strength: Level 3
>+ param name="wind_effect/strength" value="3.0"


## Acknowledgements & License
This project includes selected packages from the [MARSIM repository](https://github.com/hku-mars/MARSIM),
developed by the MARS team at The University of Hong Kong.
We sincerely thank them for their open-source contribution.

All reused code complies with the original MARSIM open-source license.
Users must adhere to the original license terms when using this project.


## Q&A

Please feel free to submit issues or start discussions.
We will respond as soon as possible after reviewing them.




