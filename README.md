# Multi-Robot Coordination
This repository contains a hierachical multi-robot global planner that generates paths for a team of Unmanned Aerial Vehicles (UAVs).
If you use this code in your academic work, please cite:

    @inproceedings{bartolomei2020multi,
      title={Multi-robot Coordination with Agent-Server Architecture for Autonomous Navigation in Partially Unknown Environments},
      author={Bartolomei, Luca and Karrer, Marco and Chli, Margarita},
      booktitle={2020 {IEEE/RSJ} International Conference on Intelligent Robots and Systems ({IROS})},
      year={2020}
    }

This project is released under a GPLv3 license.

## Video
<a href="https://www.youtube.com/watch?v=BlFbiuV-d10" target="_blank"><img src="https://img.youtube.com/vi/BlFbiuV-d10/0.jpg" alt="Mesh" width="240" height="180" border="10" /></a>

## Table of Contents
* [Project Overview](#project-overview)
* [Installation instructions](#installation-instructions)
* [Experiments](#experiments)
    * [Gazebo Powerplant Experiment](#gazebo-powerplant-experiment)
    * [Chemical Plant Experiments](#chemical-plant-experiments)  
        _1_. [Map navigation with 4 agents](#1-map-navigation-with-4-agents)  
        _2_. [Map re-use between agents](#2-map-re-use-between-agents)  
        _3_. [Planning in same area of interest](#3-planning-in-same-area-of-interest)  
* [Troubleshooting](#troubleshooting)
* [Contributing](#contributing)

## Project Overview
This repository contains all the instructions to install and run the multi-robot planning pipeline described in the paper above. In particular, the pipeline is composed of many different components that run either on the server or on the agents.  
On the server side, the main components are:
* Multi-robot Global Planner - this repository
* Pose Graph Backend - [link](https://github.com/VIS4ROB-lab/pose_graph_backend)
* Multi-agent Voxblox - [link](https://github.com/VIS4ROB-lab/voxblox_multi_agent)  

Onboard each agent, the main components are:
* Client-server version of VINS-Mono - [link](https://github.com/VIS4ROB-lab/vins_client_server)
* Local Obstacle avoidance -  this repository

The experiences collected by all the agents are sent to the central server, where an optimization-based pose-graph backend fuses all of them. The backend generates a globally consistent map of the navigation environment and localizes the agents in it. This map is used by the global planner, that coordinates the motions of the robots navigating them towards the respective goal positions.  

## Installation instructions
Install Ubuntu 18.04 and ROS Melodic. Install these dependencies:
```
$ sudo apt install git libv4l-dev libsuitesparse-dev libnlopt-dev
$ sudo apt install python-catkin-tools python-wstool ros-melodic-joy ros-melodic-octomap-ros protobuf-compiler libgoogle-glog-dev ros-melodic-mav-msgs ros-melodic-mav-planning-msgs ros-melodic-sophus ros-melodic-hector-gazebo-plugins ros-melodic-pcl-ros ros-melodic-pcl-conversions libatlas-base-dev python-matplotlib python-numpy
$ sudo apt install liblapacke-dev libode6 libompl-dev libompl12 libopenexr-dev libglm-dev
$ sudo apt install clang-format
```  

Create a `catkin_ws` folder:
```
$ mkdir -p catkin_ws/src
$ cd catkin_ws
```
Set-up the workspace:
```
$ source /opt/ros/melodic/setup.bash
$ catkin init
$ catkin config --extend /opt/ros/melodic
$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
$ catkin config --merge-devel
```

Clone the dependencies:
```
$ cd ~/catkin_ws/src
$ git clone git@github.com:VIS4ROB-lab/multi_robot_coordination.git # Https: git clone https://github.com/VIS4ROB-lab/multi_robot_coordination.git
$ wstool init
$ wstool merge multi_robot_coordination/dependencies_ssh.rosinstall # To clone with https: multi_robot_coordination/dependencies_https.rosinstall
$ wstool up -j8
```

Remove the useless packages:
```
$ touch mav_control_rw/mav_linear_mpc/CATKIN_IGNORE
$ rm -rf ewok/catkin_simple/
$ rm -rf ewok/mav_comm/
$ rm -rf ewok/rotors_simulator/
```  

Finally, build the workspace:
```
$ cd ~/catkin_ws
$ catkin build
```  

Once the building is complete, source the workspace:
```
$ source devel/setup.bash
```

## Experiments
In the following, we illustrate how to run two experiments. In the first one, the Gazebo `Powerplant` world is used, while in the second one we show how to reproduce the experiments of the paper.

### Gazebo Powerplant experiment
In this experiment, three robots have to cover a list of user-defined waypoints in the RotorS `Powerplant` Gazebo model. In this case, the state estimates are provided by Gazebo and the pose-graph back-end will not be utilized. This experiment is meant to test and showcase the global planner.  
First, start the simulation with:
```
$ roslaunch multi_robot_simulation mav_sim_powerplant_three.launch
```  
Once the simulation is started and the UAVs have been spawned, it is possible to start the global planner and the local planners for all the agents in separate terminals:
```
$ roslaunch multi_robot_global_planner mrp_global_planner_powerplant.launch
$ roslaunch agent_local_planner agent_local_planner.launch agent_id:=0
$ roslaunch agent_local_planner agent_local_planner.launch agent_id:=1
$ roslaunch agent_local_planner agent_local_planner.launch agent_id:=2
```  
To start the experiments, call the service `rosservice call /multi_robot_global_planner/plan "{}"`. It is possible to change the waypoints for the agents in the file `multi_robot_global_planner/cfg/waypoints/waypoints_powerplant.yaml`.

### Chemical Plant Experiments
In this section we show how to run the experiments reported in the paper in a photo-realistic environment using Gazebo. The model that will be used is the abandoned Chemical Plant in Rüdersdorf. A complete 3D model of the plant is available [here](https://sketchfab.com/3d-models/abandoned-chemical-plant-rudersdorf-rawscan-d2464ec36fb644f28e904e5517f1da64) - all credits go to the original author.  
In order to import the model in Gazebo, follow these instructions, after having installed ROS and Gazebo and cloned this repository in a valid catkin workspace.
```
$ cd ~/catkin_ws/src/multi_robot_coordination
$ cp resources/chemical_plant.tar.xz ~/.gazebo/models  # Copy to Gazebo models folder
$ cd ~/.gazebo/models/
$ tar -xvf chemical_plant.tar.xz  # Unzip
$ rm chemical_plant.tar.xz  # Remove zip file
```  
To test if the model has been properly set, run:
```
$ roslaunch multi_robot_simulation mav_sim_example.launch world:=chemical-plant run_gazebo_gui:=true
```
The model will take some time to load. When it is done, you should be able to see the model imported in Gazebo.  
**Note**: a decently powerful PC is required to be able to use this 3D model in Gazebo. If this model is too big, replace it with a smaller one. [This website](https://sketchfab.com/) is a good source of models.

#### Note
Running the experiments with more than one agent is not recommended on a single PC, unless you have a particularly powerful PC (since it would be running visual-inertial odometry, pointcloud filtering, Pose Graph optimization, loop detection, mesh reconstruction and path planning for every agent).  
In general, it is recommended to outsource some of the computations to external PCs. In particular, we recommend to run on a central server:
* Pose-graph backend;
* Voxblox mapping;
* Global path planning; and 
* Gazebo. 

Instead, on the single agents, run:
* Visual-Inertial state estimation (VINS-Mono);
* Local mapping;
* Local path planning; and
* MPC controller.

#### Run the simulation on multiple PCs
Here we show how to run the Gazebo simulation on two PCs. Make sure that the two computers are connected to the same network (Wi-Fi or via cable - cable recommended). First, install:
```
$ sudo apt install net-tools
```  
On the PC where you are going to run the Gazebo simulation, start a `roscore`. Then, **in all the terminals you are going to use**, export the `ROS_IP` and set the `ROS Master URI`:  
```
$ ifconfig  # to get the IP of the first machine -> here ${machine_1_ip}
$ export ROS_IP=${machine_1_ip}  
$ export ROS_MASTER_URI=http://${machine_1_ip}:11311/
```  
Then you can start the simulation on the master PC (in this example, for four agents):  
```
$ roslaunch multi_robot_simulation mav_sim_chemical_plant_four.launch 
```
In the second PC, in **all the terminals** you are going to use, make sure you **export ROS_IP of the second machine** and the **ROS Master URI of the first machine**:
```
$ ifconfig # to get the IP of the second machine -> here ${machine_2_ip}
$ export ROS_IP=${machine_2_ip}  
$ export ROS_MASTER_URI=http://${machine_1_ip}:11311/  # Note: machine 1 here!
``` 
It is now possible to run other nodes on the two PCs - all the nodes will be connected to the same `rosmaster`.

### 1. Map navigation with 4 agents
In this experiment, we show how to run the complete pipeline in order to perform a full exploration of an area of interest, selected at the beginning of the mission by the user. If multiple PCs are in use, make sure to run the right launch file on the right computer.  
First, start the simulation:
```
$ roslaunch multi_robot_simulation mav_sim_chemical_plant_four.launch
```  

Once the simulation is started (i.e. wait for all the agents to be visible in RViz), it is possible to launch VINS-Mono and the mapping pipeline for every agent. These nodes should run onboard each agent's PC. For the first agent (highest priority is assigned to agent 0):
```
$ roslaunch multi_robot_simulation vins_sim.launch agent_id:=0
$ roslaunch agent_local_planner agent_mapping_four_sim_0.launch
```
Repeat the same operation for all the agents, launching the nodes in the respective PCs:
```
$ roslaunch multi_robot_simulation vins_sim.launch agent_id:=1
$ roslaunch agent_local_planner agent_mapping_four_sim_1.launch

$ roslaunch multi_robot_simulation vins_sim.launch agent_id:=2
$ roslaunch agent_local_planner agent_mapping_four_sim_2.launch

$ roslaunch multi_robot_simulation vins_sim.launch agent_id:=3
$ roslaunch agent_local_planner agent_mapping_four_sim_3.launch
```  

On the server's side, start the pose graph and wait for the initialization to be done:
```
$ roslaunch pose_graph_backend pose_graph_node_simulation.launch num_agents:=4
```
It is possible now to launch the path-planning pipeline. On the server's PC:
```
$ roslaunch multi_robot_global_planner mrp_global_planner_chemical_plant_four.launch
```  

On the agents' PCs, run on the respective PCs:
```
$ roslaunch agent_local_planner agent_local_planner_sim.launch agent_id:=0
$ roslaunch multi_robot_simulation gps_pose_graph_initializer.launch agent_id:=0

$ roslaunch agent_local_planner agent_local_planner_sim.launch agent_id:=1
$ roslaunch multi_robot_simulation gps_pose_graph_initializer.launch agent_id:=1

$ roslaunch agent_local_planner agent_local_planner_sim.launch agent_id:=2
$ roslaunch multi_robot_simulation gps_pose_graph_initializer.launch agent_id:=2

$ roslaunch agent_local_planner agent_local_planner_sim.launch agent_id:=3
$ roslaunch multi_robot_simulation gps_pose_graph_initializer.launch agent_id:=3
```
Once the `Pose Graph` has initialized all the transformations (i.e. when the message `GPS covariance: XXX` is not printed anymore in the terminal, or when the message `Initialized the GPS reference transformation for agent X` has been printed for every agent), it is possible to run the experiment. First, initialize MSF for all the agents:
```
$ rosservice call /firefly_0/pose_sensor_vins_0/pose_sensor/initialize_msf_scale "scale: 1.0"
$ rosservice call /firefly_1/pose_sensor_vins_1/pose_sensor/initialize_msf_scale "scale: 1.0"
$ rosservice call /firefly_2/pose_sensor_vins_2/pose_sensor/initialize_msf_scale "scale: 1.0"
$ rosservice call /firefly_3/pose_sensor_vins_3/pose_sensor/initialize_msf_scale "scale: 1.0"
```
Then, start the planning:
```
$ rosservice call /multi_robot_global_planner/plan "{}"
```  
It is possible to trigger the `Return-Home` behaviour by publishing on the topic:
```
$ rostopic pub /multi_robot_global_planner/return_home std_msgs/Int16 "data: 0" -1
```
where you need to put the right agent ID in the `data` field. If you put `-1`, all the agents will return home.

### 2. Map re-use between agents
In this experiment, we showcase the advantages of the centralized path-planning pipeline, by showing how an agent can re-use the map created by another robot. Notice that, due to the stochasticity of the RRT* planner, it may happen that in some runs one of the agents may not go through the same area mapped by the other UAVs.  
First, start the simulation:
```
$ roslaunch multi_robot_simulation mav_sim_chemical_plant_exp_common_map.launch
```
Once the simulation is started (i.e. wait for all the agents to be visible in RViz), it is possible to launch VINS-Mono and the mapping pipeline for every agent. These nodes should run onboard each agent's PC. For the first agent:
```
$ roslaunch multi_robot_simulation vins_sim.launch agent_id:=0
$ roslaunch agent_local_planner agent_mapping_two_sim_0.launch
```
Repeat the same operation for the other agent:
```
$ roslaunch multi_robot_simulation vins_sim.launch agent_id:=1
$ roslaunch agent_local_planner agent_mapping_two_sim_1.launch
```  
On the server's side, start the pose graph and wait for the initialization to be done:
```
$ roslaunch pose_graph_backend pose_graph_node_simulation.launch num_agents:=2
```
It is possible now to launch the path-planning pipeline. On the server's PC:
```
$ roslaunch multi_robot_global_planner mrp_global_planner_chemical_plant_exp_common_map.launch
```
On the agents' PCs, run on the respective PCs:
```
$ roslaunch agent_local_planner agent_local_planner_sim.launch agent_id:=0
$ roslaunch multi_robot_simulation gps_pose_graph_initializer.launch agent_id:=0

$ roslaunch agent_local_planner agent_local_planner_sim.launch agent_id:=1
$ roslaunch multi_robot_simulation gps_pose_graph_initializer.launch agent_id:=1
```
Once the `Pose Graph` has initialized all the transformations (i.e. when the message `GPS covariance: XXX` is not printed anymore in the terminal, or when the message `Initialized the GPS reference transformation for agent X` has been printed for every agent), it is possible to run the experiment. First, initialize MSF for all the agents:
```
$ rosservice call /firefly_0/pose_sensor_vins_0/pose_sensor/initialize_msf_scale "scale: 1.0"
$ rosservice call /firefly_1/pose_sensor_vins_1/pose_sensor/initialize_msf_scale "scale: 1.0"
```
Then, start the planning:
```
$ rosservice call /multi_robot_global_planner/plan "{}"
```  
It is possible to trigger the `Return-Home` behaviour by publishing on the topic:
```
$ rostopic pub /multi_robot_global_planner/return_home std_msgs/Int16 "data: 0" -1
```
where you need to put the right agent ID in the `data` field. If you put `-1`, all the agents will return home.

### 3. Planning in same area of interest
In the last experiment, we show the performace of the global planner when three agents have to navigate in the same area of interest.
First, start the simulation:
```
$ roslaunch multi_robot_simulation mav_sim_chemical_plant_three_twist.launch
```
Once the simulation is started (i.e. wait for all the agents to be visible in RViz), it is possible to launch VINS-Mono and the mapping pipeline for every agent. These nodes should run onboard each agent's PC. For the first agent:
```
$ roslaunch multi_robot_simulation vins_sim.launch agent_id:=0
$ roslaunch agent_local_planner agent_mapping_three_sim_0.launch
```
Repeat the same operation for all the agents, launching the nodes in the respective PCs:
```
$ roslaunch multi_robot_simulation vins_sim.launch agent_id:=1
$ roslaunch agent_local_planner agent_mapping_three_sim_1.launch

$ roslaunch multi_robot_simulation vins_sim.launch agent_id:=2
$ roslaunch agent_local_planner agent_mapping_three_sim_2.launch
```  
On the server's side, start the pose graph and wait for the initialization to be done:
```
$ roslaunch pose_graph_backend pose_graph_node_simulation.launch num_agents:=3
```
It is possible now to launch the path-planning pipeline. On the server's PC:
```
$ roslaunch multi_robot_global_planner mrp_global_planner_chemical_plant_three_twist.launch
```
On the agents' PCs, run on the respective PCs:
```
$ roslaunch agent_local_planner agent_local_planner_sim.launch agent_id:=0
$ roslaunch multi_robot_simulation gps_pose_graph_initializer.launch agent_id:=0

$ roslaunch agent_local_planner agent_local_planner_sim.launch agent_id:=1
$ roslaunch multi_robot_simulation gps_pose_graph_initializer.launch agent_id:=1

$ roslaunch agent_local_planner agent_local_planner_sim.launch agent_id:=2
$ roslaunch multi_robot_simulation gps_pose_graph_initializer.launch agent_id:=2
```
Once the `Pose Graph` has initialized all the transformations (i.e. when the message `GPS covariance: XXX` is not printed anymore in the terminal, or when the message `Initialized the GPS reference transformation for agent X` has been printed for every agent), it is possible to run the experiment. First, initialize MSF for all the agents:
```
$ rosservice call /firefly_0/pose_sensor_vins_0/pose_sensor/initialize_msf_scale "scale: 1.0"
$ rosservice call /firefly_1/pose_sensor_vins_1/pose_sensor/initialize_msf_scale "scale: 1.0"
$ rosservice call /firefly_2/pose_sensor_vins_2/pose_sensor/initialize_msf_scale "scale: 1.0"
```
Then, start the planning:
```
$ rosservice call /multi_robot_global_planner/plan "{}"
```  
It is possible to trigger the `Return-Home` behaviour by publishing on the topic:
```
$ rostopic pub /multi_robot_global_planner/return_home std_msgs/Int16 "data: 0" -1
```
where you need to put the right agent ID in the `data` field. If you put `-1`, all the agents will return home. Make sure that the area around the starting position of the robots has been mapped and inserted in the Voxblox map.

## Troubleshooting
* **OMPL**: If there are problems with `OMPL`, make sure that the `ROS` version is not installed: `sudo apt remove ros-melodic-ompl`.  
* **Simulation Test**: If in the experiments the global planner outputs `[MR Global Planner] The map is empty, cannot plan`, it means that Voxblox has not received any point cloud from the `Pose Graph` yet. To solve this issue, wait a few seconds and then trigger the global planner again.
* **Gazebo Simulation**: If the simulation is slow, replace the Chemical Plant model with a smaller one. This will require the generation of a new set of waypoints for the agents. [This website](https://sketchfab.com/) is a good source of models.
* **Return Home**: If the planner cannot find the path towards the home position for one agent, it is likely that the home position has not been inserted in the Voxblox map. To solve this issue, make sure that the starting positions of the agents are mapped in Voxblox.

## Contributing
Contributions that help to improve the code are welcome. In case you want to contribute, please adapt to the [Google C++ coding style](https://google.github.io/styleguide/cppguide.html/) and run `bash clang-format-all .` on your code before any commit.
