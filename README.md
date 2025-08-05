# 2025 Aerial Autonomy Challenge 
# 2025 IROS COMPETITION


## Overview

IROS 2025 will be held from **October 19 to 25** in **Hangzhou, China**, bringing together thousands of leading scholars, technology experts, and industry leaders from around the world to witness the latest innovations in robotics and explore the future of the industry.

Differential Robotics is organizing the **Aerial Autonomy Challenge** as part of the IROS 2025 competition track. The challenge aims to evaluate the autonomous navigation capabilities of aerial robots in unknown and complex environments, including structured maps, dynamic obstacles, simulated forests, wind disturbances, and narrow gaps.

## Challenge Objectives

The **Aerial Autonomy Challenge** comprises five designed scenarios aimed at systematically evaluating the capabilities of autonomous aerial robots in dynamic environments. The tasks focus on key aspects such as localization, perception, decision-making, planning, and control. By closely emulating real-world operational demands, the challenge serves as a rigorous benchmark and a technology validation platform for advancing the deployment of aerial robotics in multi-scenario, cross-domain applications.

<p align="center">
  <img src="image/image_1.jpg" alt="alt text" width="350" />
</p>


The competition is now open for registration! We sincerely invite researchers and engineering practitioners from around the world to join us in this exciting competition, to drive breakthroughs in aerial robotics and accelerate the evolution of autonomous aerial systems!

Competition Official Web: https://www.nspacerobot.com/iroschallenge/  
Official Discord Channel: https://discord.gg/spgGRz3w

## About the Simulator

**Differential Robotics** provides the official simulator for this competition, which is used for algorithm validation and venue simulation testing before the event. The simulator accurately reproduces typical environments found in the Aerial Autonomy Challenge, including forests, dynamic obstacles, wind disturbances, narrow gaps, and other complex scenarios.

The simulator also offers interfaces for radar point clouds, first-person views from the aerial vehicle, and more. It is expected to serve as the platform for the **Aerial Autonomy Challenge** preliminary rounds in the future.


## Official Test Environment
> ros-noetic  
> ubuntu20.04  
> NVIDIA RTX4060
> INTEL I7

## Download Simulator and Demo Code
#### Download Demo Code
>+ git clone https://github.com/DifferentialRobotics/Aerial_Autonomy_Challenge.git
#### Download Simulator（Please download the latest version）
Please refer to the following files:
[README_Simulator_CN.md](AerialAutonomyChallenge-Simulator/README_Simulator_CN.md) OR [README_Simulator.md](AerialAutonomyChallenge-Simulator/README_Simulator.md)


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
>+ Press the Esc key to show the mouse cursor. In mouse-visible mode, press the Win key to right-click the simulator icon and close the simulator (or use Alt + F4 to close the simulator directly).

## ROS Topic Interaction
#### Published Topics:  
>+ /drone_0_pcl_render_node/cloud
>+ /image/compressed

#### Subscribed Topics:  
>+ /quad_0/lidar_slam/odom

## Scenario Parameter Configuration 
Wind field strength is configurable in：'[run_in_sim.xml](AerialAutonomyChallenge-Demo-EgoPlannerv2/src/planner/plan_manage/launch/include/run_in_sim.xml)'
#### Current Wind Strength: Level 3
>+ param name="wind_effect/strength" value="3.0"


## Acknowledgements & License
During the development of this project, we referenced and utilized several open-source packages from [EGO-Planner-v2](https://github.com/hku-mars/MARSIM) and [MARSIM](https://github.com/hku-mars/MARSIM).  
We would like to express our sincere gratitude to the **FAST-Lab at Zhejiang University** and the **MARS team at The University of Hong Kong** for their valuable open-source contributions.

All related code is strictly used in compliance with the original project's open-source license agreements. When utilizing this project, users must adhere to the terms of the respective licenses.

## Q&A

Please feel free to submit issues or start discussions.
We will respond as soon as possible after reviewing them.




