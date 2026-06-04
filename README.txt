# Autonomous Navigation System for TurtleBot3 (ROS2 + Nav2)

## Overview

This project implements an autonomous navigation stack for TurtleBot3 using ROS2 Humble and Nav2 inside a Docker-based simulation environment.

The system extends a standard TurtleBot3 navigation setup with:
- Adaptive navigation controller
- Custom metric collection system
- Runtime performance logging
- Foxglove-based visualization
- Behavior Tree monitoring integration

The goal is to improve observability and robustness of autonomous navigation while maintaining compatibility with the official TurtleBot3 simulation stack.
The implementation was developed in a custom workspace while maintaining full compatibility with the provided base simulation stack.

---

## Base System

The project is built on top of the official TurtleBot3 simulation environment:

- Docker image: `merabro/turtlebot3-sim:latest`
- ROS2 distribution: Humble
- Navigation stack: Nav2 (planner, controller, BT navigator)
- Simulation: TurtleBot3 Gazebo environment

Compatibility with the base stack is preserved to ensure reproducibility across environments.
The docker image name is ros2_humble_dev and the docker container is ros2_humble_new.

---

## System Architecture

### Main Components

- `adaptive_controller_node`
  - Dynamically adapts Nav2 parameters during runtime

- `metric_collector_node`
    -The following node is a LifeCycle node that does:
        - Calculation of the distance travelled
        - Evaluation of the Battery consumption (simulated)
        - Minimum obstacle distance
        - Recovery behavior count
        - Goal reached flag

- `data_logger_node`
  - Logs navigation metrics in CSV format for offline analysis

- Nav2 stack:
  - `planner_server`
  - `controller_server`
  - `bt_navigator`
  - `costmap_2d` (global & local)

---

## TF Frame Structure

The system follows a standard ROS TF tree:

Correct TF consistency is critical for:
- localization stability
- costmap correctness
- planner reliability

---

## Docker Setup

### Clean up of previous container and images
docker stop ros2_humble_new
docker rm ros2_humble_new
docker rmi ros2_humble_dev

# Build and Start the container and the image
docker-compose build
docker-compose up -d

# Verify it worked
docker logs ros2_humble_new

# Enter in the container
docker exec -it ros2_humble_new bash

Configuration (Nav2 Parameters)

The system uses a custom Nav2 configuration file:

config/params.yaml

Key tuned components:

Planner frequency
Controller rate
Costmap resolution and inflation
Goal tolerance parameters

These parameters are dynamically adjusted by the adaptive controller.

Metrics & Logging

The system collects runtime metrics:

distance to goal
navigation success/failure
control loop performance
planner execution rate
costmap updates

Data is exported in CSV format for offline evaluation.

Visualization

The system is integrated with Foxglove Studio for real-time monitoring:

/tf, /odom, /map
/plan
/navigation_metrics
/rosout
behavior tree logs
Experimental Results
Environment 1: Open space
High success rate
Stable planning and control performance
Environment 2: Obstacle-rich map
Increased replanning frequency observed
occasional control loop delays
Environment 3: Dynamic navigation tuning
Adaptive controller improves response time
costmap resizing impacts planner stability (observed trade-off)
Known Limitations
Planner sensitivity to costmap resizing frequency
occasional "No valid trajectories" under dense obstacles
control loop misses target frequency in complex scenarios
Nav2 aborts followed by automatic replanning (expected behavior)
Design Decisions
Multi-node architecture for modularity
Separation between metric collection and control logic
Runtime adaptation instead of static tuning
Docker-based reproducibility for consistent execution
Conclusion

This project demonstrates a modular and observable navigation system built on ROS2 Nav2, with emphasis on:

runtime metrics
adaptive behavior
system observability
reproducible deployment via Docker
Demo Video

👉 [Insert 5-minute demo video link here]