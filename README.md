# Multi-vehicle Software-In-the-loop simulation framework for formation control for Quadrotor UAV.
We provide a Robot operating system (ROS) package for controlling multiple vehilce to achieve desired shaped in gazebo based simulation that utilizes pixhawk autopilot with IRIS quadrotor and MAVROS for connection between subsystems. This package in particular subscribes vehicle state data and synchronizes the origin frame for all the vehicles for implementing the formation control guidance algorithm and publishes velocity setpoints to mavros topic '/uav0/mavros/setpoint_position/local' for executing control actuation by the autopilot. The main code can be found in /scripts/offb_node.cpp and
## Installation
Clone the package to your catkin workspace and catkin_make. The destails for setting up the ROS and gazebo based SITL simulation an be found at [pixhawk documentation](https://docs.px4.io/main/en/simulation/multi-vehicle-simulation.html):
## Implementation
1) Start the simulation:
   - cd [your firmware location]
   - source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
   - export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
   - roslaunch px4 multi_uav_mavros_sitl.launch
2) Running the Formation control pipeline:
   - cd [your catkinworkspace]
   - source /devel/setup.bash
   - rosrun uav_control offb_node
## Configuration
- Adjust formation parameters (del_x, del_y, del_z) in the main control file (offb_node.cpp) to customize formation patterns according to your application requirements.
- Fine-tune control gains (K1, K2, K3) to optimize drone performance and stability based on specific environmental conditions and mission objectives.
