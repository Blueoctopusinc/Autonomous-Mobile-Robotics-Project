# Autonomous Mobile Robotics Project
Python project created for third year robotics module using the ROS framework and designed to be ran on a TurtleBot, excerpt from presentation below explains some functionality.


# Autonomous Mobile Robotics Project - High-Level Overview

Robot control and navigation system that leverages the ROS (Robot Operating System) framework. It encompasses a range of functionalities designed to assist in robot navigation, object detection, image processing, and more.

## Key Features

### Initialization
Initializes the system by setting up ROS communication, subscribing to data topics, and initializing the navigation stack.

### Image Processing & Object Detection
Processes camera images, applies color detection to identify specific objects, and aligns the robot's camera for accurate object detection.

### Navigation
Manages the robot's navigation by sending goals to the navigation stack. Utilizes a point cloud of the environment to create a 2D occupancy grid, performing various image processing operations to deduce the centroid of each room as the start point for navigation.

### Callback Functions
Defines callbacks to handle various incoming ROS messages, facilitating real-time interaction with sensors and other devices.

### Coordinate Transformation & Utility Functions
Includes methods for coordinate conversions and provides utility functions for various operations like rotation, image manipulation, etc.

### Goal Completion Handling
Ensures appropriate actions are taken after navigation goals are completed, canceled, or aborted.

### Main Logic Loop
The core logic of the system, constantly checking for object detection and executing alignment, navigation, and detection tasks accordingly.

##Authors
Joshua Yeomans-Gladwin

