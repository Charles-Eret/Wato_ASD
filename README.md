# WATonomous ASD Admissions Assignment
This repo follows the WATonomous ASD Admission Assignment. Link to Onboarding Assignment: https://wiki.watonomous.ca/
This project implements a modular autonomous navigation stack using **ROS 2**, enabling a mobile robot to autonomously navigate to user-defined goals while avoiding dynamic obstacles. The stack includes:

- **Costmap Node**: Subscribes to `/lidar` data and generates a local occupancy grid with obstacle inflation.
- **Map Memory Node**: Aggregates local costmaps into a persistent global map using odometry and fuses updates based on robot motion thresholds.
- **Planner Node**: Performs global path planning using the A* algorithm, dynamically replanning in response to map changes or path deviations.
- **Control Node**: Uses a Pure Pursuit Controller to follow the global path in real-time by converting waypoints and odometry into velocity commands.

The system is designed to operate in a **closed-loop simulation** using **Ignition Gazebo** and **Foxglove**, integrating perception, planning, and control into a cohesive navigation pipeline. The architecture is modular, making it extensible for future integration with SLAM, dynamic obstacle tracking, or multi-robot coordination.

## Features

- Real-time occupancy grid generation from `LaserScan` and `Odometry`
- Persistent map fusion with motion-triggered updates
- Dynamic A* pathfinding on live occupancy maps
- Pure Pursuit path tracking for smooth goal convergence
- Visualization and debugging through Foxglove Studio
