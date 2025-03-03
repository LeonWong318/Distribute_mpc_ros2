# Toward Efficient Collaboration in Autonomous Mobile Robot Fleets: Addressing Latency and Distributed Model Predictive Control

This is the master thesis repository collaborated by Yinsong and Zihao.

## System Requirements

### Prerequisites
- Operating System: Ubuntu 20.04
- ROS2: foxy
- Python: 3.8
- OpEn: Optimization Engine for MPC solver ([Installation Guide](https://alphaville.github.io/optimization-engine/docs/installation))

## Env set up:
recommend using conda for environment:
```
conda create -n ros_dmpc python=3.8
```
```
conda activate ros_dmpc
```
```
pip install -r requirements.txt
```
```
sudo apt install ros-foxy-gazebo-ros-pkgs ros-foxy-ros-core ros-foxy-geometry2
```

## MPC solver build:
```
python src/build_solver.py
```
That can ensure mpc_solver/ located in project root folder.


## Start Simulation:
```
chmod +x .
```
```
./run_cluster.sh
```
## Launch gazebo:
'''
colcon build
source install/setup.bash
ros2 launch pkg_gazebo_simulation gazebo_world.launch.py
'''
