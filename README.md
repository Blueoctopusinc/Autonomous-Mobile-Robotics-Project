# Autonomous Mobile Robotics Project
Python project created for third year robotics module using the ROS framework and designed to be ran on a TurtleBot, excerpt from presentation below explains some functionality.
![Presentation Image](https://i.ibb.co/BB7n01k/Screenshot-from-2019-08-07-16-45-54.png)


Certainly, here's the updated README template with a "Technologies" section listing the main technologies used in your single-file Python application:

# Object Finder Robot

This Python application is designed for a robot that uses the ROS (Robot Operating System) framework to navigate an environment, locate objects based on color detection, and perform various actions. The robot utilizes camera images, laser scans, and odometry data for navigation and object detection.

## Functionality

- The application subscribes to camera images, laser scans, and odometry data.
- It performs color thresholding to detect objects of specific colors (blue, green, red, yellow).
- The robot navigates toward detected objects, aligns itself, and calculates distances.
- Objects within a certain distance are considered found.
- The robot attempts to align with objects before confirming their presence.


## Prerequisites

- Python 2.7 (Python 3.x is not supported due to ROS compatibility)
- ROS (Robot Operating System) installed
- OpenCV
- numpy
- math
- actionlib
- sensor_msgs
- nav_msgs
- geometry_msgs
- cv_bridge
- tf

## Technologies Used

- **ROS (Robot Operating System):** A flexible framework for writing robot software. It provides various tools, libraries, and conventions that simplify the development of complex robotics applications.

- **OpenCV:** An open-source computer vision and machine learning software library. It provides tools for image processing, object detection, and computer vision tasks.

- **numpy:** A powerful library for numerical computations in Python. It provides support for arrays, matrices, and various mathematical functions.

- **actionlib:** A ROS package that provides a standardized interface for creating and managing actions in a distributed system.

- **sensor_msgs:** A set of messages that provide data from sensors and devices.

- **nav_msgs:** Message types used in navigation and motion planning in ROS.

- **geometry_msgs:** Message types for representing geometric shapes, poses, and transformations.

- **cv_bridge:** A ROS library that provides a bridge between ROS messages and OpenCV.

- **tf:** The transform library in ROS, used for working with coordinate frame transformations.

## Installation

1. Clone the repository:

   ```
   git clone <repository_url>
   ```

2. Build the ROS workspace (if not already done):

   ```
   cd <path_to_workspace>
   catkin_make
   ```

3. Source the workspace:

   ```
   source <path_to_workspace>/devel/setup.bash
   ```

## Usage

1. Run ROS master:

   ```
   roscore
   ```

2. Launch required nodes:

   ```
   roslaunch <your_robot_description_package> <launch_file>.launch
   ```

3. Run the Python application:

   ```
   python object_finder.py
   ```

---


##Authors
Joshua Yeomans-Gladwin

