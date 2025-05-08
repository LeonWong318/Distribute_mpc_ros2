# Toward Efficient Collaboration in Autonomous Mobile Robot Fleets: Addressing Latency and Distributed Model Predictive Control

This repository contains the master thesis project developed by Yinsong and Zihao, focusing on distributed model predictive control for autonomous mobile robot fleets.

## System Requirements

### Prerequisites
- Operating System: Ubuntu 20.04
- ROS2: Foxy
- Python: 3.8
- OpEn: Optimization Engine for MPC solver ([Installation Guide](https://alphaville.github.io/optimization-engine/docs/installation))

## Setup Instructions

### Environment Setup
We recommend using Conda for environment management:
```bash
# Create and activate a new Conda environment
conda create -n ros_dmpc python=3.8
conda activate ros_dmpc

# Install Python dependencies
pip install -r requirements.txt

# Install required ROS packages
sudo apt install ros-foxy-gazebo-ros-pkgs ros-foxy-ros-core ros-foxy-geometry2
```

### MPC Solver Configuration
Build the MPC solver with default parameters:
```bash
python src/build_solver.py
```
This will create the `mpc_solver/` directory in the project root folder.

To modify the MPC solver parameters:
1. Edit the parameters in `./config/mpc_fast.yaml`
2. Rebuild the solver:
```bash
rm -rf ./mpc_solver/
python src/build_solver.py
```
> **Note:** When changing the `mpc_ts` parameter, ensure it matches the `ts` parameter in `mpc_fast.yaml`.

## Running Simulations

### Single Simulation Run
You can use the pre-configured system parameters in `./config/sys_config.yaml` or customize them for testing. Parameter changes here don't require rebuilding the solver (except for `mpc_ts` as noted above).

```bash
chmod +x run_cluster.sh
./run_cluster.sh
```

### Batch Simulation Testing
For multiple simulation runs with varying parameters:

1. Configure test parameters in `./config/test_config.yaml`:
   - `latency_values`: Network latency values (in seconds)
   - `iterations_per_latency`: Number of iterations per latency value on each map
   - `timeout_seconds`: Single test timeout duration

2. Run the batch tests:
```bash
chmod +x run_test.sh
./run_test.sh
```

The test module will automatically perform multiple tests based on these settings and save results in a new folder in the root directory.

> **Note:** When modifying or adding maps, ensure that `name`, `map_path`, `graph_path`, `robot_start_path`, `robot_spec_path`, `world_file`, and `rviz_config_path` are consistent with each other.

### Cross-Platform Simulation
For distributed simulations across multiple computers in the same LAN:

On the first computer:
```bash
chmod +x run_cluster.sh
./run_cluster.sh
```

On the second computer:
```bash
chmod +x run_local_sim.sh
./run_local_sim.sh
```

The two simulation environments will automatically communicate over the LAN.