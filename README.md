# ROS2 Prius Autonomous Driving (Gazebo)

This project demonstrates a teach-and-repeat autonomous driving system using ROS2 and Gazebo.

## Features
- Odometry-based localization
- Path recording from manual driving
- Autonomous path replay
- Closed-loop heading control

## How it works
1. Record path using:
   ros2 run prius_controller record_path

2. Replay autonomously:
   ros2 run prius_controller follow_path

## Tech Stack
- ROS2 (Jazzy)
- Gazebo
- Python
- able to turn initial corner on racetrack, current progress stops there
