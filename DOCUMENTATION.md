# Documentation of ROS Nodes

## Overview

This document provides detailed documentation of the ROS nodes in the `RoBorregos/home-manipulation` repository. It includes the purpose and usage of each node, a detailed explanation of the key technologies used, and examples and use cases for each technology.

## ROS Nodes

### `cartesian_movement_services`

#### `arm_movements.py`

**Purpose and Usage:**
The `arm_movements.py` node is responsible for controlling the arm movements of the robot. It provides services for moving the arm to specific Cartesian coordinates.

**Key Technologies:**
- `rospy`: Used for ROS communication.
- `xarm_msgs`: Used for controlling the xArm robot.
- `math`: Used for mathematical calculations.

**Examples and Use Cases:**
- Example of using `rospy` to create a ROS service for arm movements.
- Example of using `xarm_msgs` to send commands to the xArm robot.

#### `cartesian_client.py`

**Purpose and Usage:**
The `cartesian_client.py` node is responsible for sending requests to the `cartesian_server.py` node to move the arm to specific Cartesian coordinates.

**Key Technologies:**
- `rospy`: Used for ROS communication.
- `xarm_msgs`: Used for controlling the xArm robot.

**Examples and Use Cases:**
- Example of using `rospy` to create a ROS client for arm movements.
- Example of using `xarm_msgs` to send commands to the xArm robot.

#### `cartesian_server.py`

**Purpose and Usage:**
The `cartesian_server.py` node is responsible for handling requests from the `cartesian_client.py` node and moving the arm to specific Cartesian coordinates.

**Key Technologies:**
- `rospy`: Used for ROS communication.
- `xarm_msgs`: Used for controlling the xArm robot.

**Examples and Use Cases:**
- Example of using `rospy` to create a ROS server for arm movements.
- Example of using `xarm_msgs` to send commands to the xArm robot.

### `frida_arm_joints_server`

#### `arm_joint_server.py`

**Purpose and Usage:**
The `arm_joint_server.py` node is responsible for controlling the arm joints and gripper of the robot. It provides services for moving the arm joints to specific positions.

**Key Technologies:**
- `moveit_commander`: Used for planning and executing arm movements.
- `actionlib`: Used for handling action servers.
- `geometry_msgs`: Used for representing poses and points.

**Examples and Use Cases:**
- Example of using `moveit_commander` to plan and execute a simple arm movement.
- Example of using `actionlib` to create an action server for controlling the arm joints.

### `object_detector_3d`

#### `Clustering_Server.py`

**Purpose and Usage:**
The `Clustering_Server.py` node is responsible for processing point cloud data and performing clustering to detect objects in 3D space.

**Key Technologies:**
- `numpy`: Used for numerical operations.
- `sklearn`: Used for clustering algorithms.
- `matplotlib`: Used for visualizing point clouds.

**Examples and Use Cases:**
- Example of using `numpy` to process point cloud data.
- Example of using `sklearn` to perform k-means clustering.
- Example of using `matplotlib` to visualize the clusters.

### `pick_and_place`

#### `cartesianManipulationServer.py`

**Purpose and Usage:**
The `cartesianManipulationServer.py` node is responsible for handling requests for pick and place operations and moving the arm to specific Cartesian coordinates.

**Key Technologies:**
- `moveit_commander`: Used for planning and executing pick and place operations.
- `actionlib`: Used for handling action servers.
- `geometry_msgs`: Used for representing poses and points.

**Examples and Use Cases:**
- Example of using `moveit_commander` to plan and execute a pick and place operation.
- Example of using `actionlib` to create an action server for handling the pick and place process.

#### `getState.py`

**Purpose and Usage:**
The `getState.py` node is responsible for retrieving the current state of the robot and its environment.

**Key Technologies:**
- `rospy`: Used for ROS communication.
- `geometry_msgs`: Used for representing poses and points.

**Examples and Use Cases:**
- Example of using `rospy` to create a ROS service for retrieving the robot's state.
- Example of using `geometry_msgs` to represent the robot's state.

#### `manipulationCaller.py`

**Purpose and Usage:**
The `manipulationCaller.py` node is responsible for sending requests to the `manipulationServer.py` node to perform manipulation tasks.

**Key Technologies:**
- `rospy`: Used for ROS communication.
- `geometry_msgs`: Used for representing poses and points.

**Examples and Use Cases:**
- Example of using `rospy` to create a ROS client for manipulation tasks.
- Example of using `geometry_msgs` to represent the manipulation tasks.

#### `manipulationClient.py`

**Purpose and Usage:**
The `manipulationClient.py` node is responsible for sending requests to the `manipulationServer.py` node to perform manipulation tasks.

**Key Technologies:**
- `rospy`: Used for ROS communication.
- `geometry_msgs`: Used for representing poses and points.

**Examples and Use Cases:**
- Example of using `rospy` to create a ROS client for manipulation tasks.
- Example of using `geometry_msgs` to represent the manipulation tasks.

#### `manipulationServer.py`

**Purpose and Usage:**
The `manipulationServer.py` node is responsible for handling requests for manipulation tasks and moving the arm to specific positions.

**Key Technologies:**
- `moveit_commander`: Used for planning and executing manipulation tasks.
- `actionlib`: Used for handling action servers.
- `geometry_msgs`: Used for representing poses and points.

**Examples and Use Cases:**
- Example of using `moveit_commander` to plan and execute a manipulation task.
- Example of using `actionlib` to create an action server for handling the manipulation process.

### `pouring_services`

#### `pouring_srv.py`

**Purpose and Usage:**
The `pouring_srv.py` node is responsible for handling requests for pouring tasks and controlling the arm to perform the pouring action.

**Key Technologies:**
- `rospy`: Used for ROS communication.
- `geometry_msgs`: Used for representing poses and points.

**Examples and Use Cases:**
- Example of using `rospy` to create a ROS service for pouring tasks.
- Example of using `geometry_msgs` to represent the pouring tasks.
