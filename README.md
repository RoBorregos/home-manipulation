# home-manipulation

## Description
This repository contains the ROS workspace and tools needed to run manipulation tasks for the RoBorregos @Home robot. It includes the robot description with URDF and mesh files for the most recent robot including a EAI Dashgo B1 mobile base and a UFactory xArm6. It also contains the MoveIt configuration and packages required for picking, placing and manipulating objects, including 2D and 3D object detector, pointcloud processing and the manipulation pipelines for diverse tasks.

## Index

- [Installation](#installation)
- [Docker Installation](#docker-installation)
- [Usage](#usage)
- [Purpose of the Area in the Service Robot](#purpose-of-the-area-in-the-service-robot)
- [Technologies Used in Each ROS Node](#technologies-used-in-each-ros-node)
- [Examples and Use Cases](#examples-and-use-cases)

## Installation

To run the repository, ROS1 Noetic is required. To install ROS Noetic, follow the instructions in the [official ROS website](http://wiki.ros.org/noetic/Installation/Ubuntu).

To install ros dependencies from the workspace, run the following commands:
```bash
cd ws/
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Additional dependencies are required for the xArm submodules and the Grasping Pose Detector. For xArm, follow the instructions in the [xArm ROS repository](https://github.com/xArm-Developer/xarm_ros#3-preparations-before-using-this-package), and for the Grasping Pose Detector, follow the instructions in the [Grasping Pose Detector repository](https://github.com/atenpas/gpd#install). For GPD, install eigen and pcl with the following commands:
```bash
sudo apt update && apt install -y libeigen3-dev libpcl-dev
```

## Docker Installation
The repository contains a Makefile and scripts for running the workspace in a Docker container. It also allows for use with CUDA (which helps in running the object detection YOLO models). 

### Docker Pre-requisites
- [Docker Engine](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
- [Post-installation steps for Linux](https://docs.docker.com/engine/install/linux-postinstall/)
If using GPU:
- NVIDIA Driver 
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/index.html))

### Image and Container Creation
To create the Docker image, run the following command:
```bash
make manipulation.build # for CPU
make manipulation.build.cuda # for GPU
```

To create the Docker container, run the following command:
```bash
make manipulation.create # for CPU
make manipulation.create.cuda # for GPU
```

### Running the Container
To run the Docker container, run the following command:
```bash
make manipulation.up
make manipulation.shell # for GPU
```

### Removing the Container
To remove the Docker container, run the following command:
```bash
make manipulation.down
make manipulation.remove
```

## Usage

### Robot Description
The robot description is located in the `ws/src/robot_description` package. It contains the URDF and mesh files for the robot, including the mobile base and arm. After sourcing the workspace, visualize the robot in RViz with the following command:
```bash
roslaunch robot_description display.dashgo.launch
```

### MoveIt Configuration
To turn on the xArm6 and allow the robot to make movements with MoveIt planning and other planning and movement options, run the following command:
```bash
roslaunch dashgo_moveit_config dashgo_xarm.launch robot_ip:=<robot_ip>
```

### Manipulation Pipelines
The current, most recent and stable manipulation pipelines are located in the `ws/src/pick_and_place` package. The pipelines include the object detection, pointcloud processing and manipulation tasks. To run the manipulation pipeline, run the following command:
```bash
roslaunch pick_and_place cartesian.manipulation.launch
```

This command also requires the ZED2 camera, or any other camera that publishes a pointcloud in the `/zed2/zed_node/point_cloud/cloud_registered` topic. The camera can be opened with the following command from the [zed-ros-wrapper](https://github.com/RoBorregos/zed-ros-wrapper) repository in the machine where the camera is connected:
```bash
roslaunch zed_wrapper zed2_robot.launch
```

## Purpose of the Area in the Service Robot

The purpose of the area in the service robot is to provide a functional and efficient environment for performing various manipulation tasks. The area is designed to accommodate the robot's hardware components, including the mobile base, arm, and sensors, and to facilitate the execution of tasks such as picking, placing, and manipulating objects. The area is also equipped with the necessary software modules and tools to enable seamless integration and coordination of the robot's components, ensuring reliable and accurate performance.

## Technologies Used in Each ROS Node

### `cartesian_movement_services`
- `rospy`: Used for ROS communication and service handling.
- `xarm_msgs`: Used for controlling the xArm robot.
- `math`: Used for mathematical calculations.

### `frida_arm_joints_server`
- `moveit_commander`: Used for planning and executing arm movements.
- `actionlib`: Used for handling action servers.
- `geometry_msgs`: Used for representing poses and points.

### `object_detector_3d`
- `numpy`: Used for numerical operations.
- `sklearn`: Used for clustering algorithms.
- `matplotlib`: Used for visualizing point clouds.

### `pick_and_place`
- `moveit_commander`: Used for planning and executing pick and place operations.
- `actionlib`: Used for handling action servers.
- `geometry_msgs`: Used for representing poses and points.

### `pouring_services`
- `rospy`: Used for ROS communication and service handling.
- `xarm_msgs`: Used for controlling the xArm robot.
- `math`: Used for mathematical calculations.

## Examples and Use Cases

### `cartesian_movement_services`
- Example: Using `rospy` to create a ROS service for arm movements.
- Use Case: Sending commands to the xArm robot using `xarm_msgs`.

### `frida_arm_joints_server`
- Example: Using `moveit_commander` to plan and execute a simple arm movement.
- Use Case: Creating an action server for controlling the arm joints using `actionlib`.

### `object_detector_3d`
- Example: Using `numpy` to process point cloud data.
- Use Case: Performing k-means clustering using `sklearn` and visualizing the clusters using `matplotlib`.

### `pick_and_place`
- Example: Using `moveit_commander` to plan and execute a pick and place operation.
- Use Case: Creating an action server for handling the pick and place process using `actionlib`.

### `pouring_services`
- Example: Using `rospy` to create a ROS service for pouring services.
- Use Case: Sending commands to the xArm robot using `xarm_msgs` for pouring operations.
