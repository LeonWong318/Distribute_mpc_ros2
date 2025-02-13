# Toward Efficient Collaboration in Autonomous Mobile Robot Fleets: Addressing Latency and Distributed Model Predictive Control

This is the master thesis repository collaborated by Yinsong and Zihao.

## System Requirements

### Prerequisites
- Operating System: Ubuntu 20.04
- ROS2: foxy
- Python: 3.8
- OpEn: Optimization Engine for MPC solver ([Installation Guide](https://alphaville.github.io/optimization-engine/docs/installation))

## ROS2 nodes build:
If you want to rebuild the node:
```
rm -rf build/ install/ log/
```
After that,
```
colcon build
```
```
source install/setup.bash
```

## Run robot manager
```
ros2 launch manager manager.launch.py
```

## Run robot
```
ros2 launch robot robot.launch.py robot_id:=1
```

## Echo
```
ros2 topic echo /robot_1/state
```
