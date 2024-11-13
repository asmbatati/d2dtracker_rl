# d2dtracker_rl

This repository contains reinforcement learning (RL) code for the iHunter project, developed as part of the Robotics & Internet of Things Lab (RIOTU) at Prince Sultan University. The RL module enables autonomous control of a drone in Gazebo Garden, with reinforcement learning algorithms implemented using Stable Baselines3 and Gymnasium. 

The main repository for iHunter is located here: [d2dtracker_sim](https://github.com/mzahana/d2dtracker_sim).

## Dependencies

This project requires the following dependencies:
- **ROS 2 Humble**: Robot Operating System (ROS) framework for communication and control.
- **Gazebo Garden**: Simulation environment for robotics applications.
- **PX4**: Autopilot software to enable control of the drone.
- **Stable Baselines3**: Library for state-of-the-art reinforcement learning algorithms.
- **Gymnasium**: Interface for building and interacting with RL environments, compatible with Stable Baselines3.
## Usage

### Step 1: Launch the Environment

To initialize the simulation environment, run:
```bash
ros2 launch d2dtracker_rl env.launch.py
Step 2: Start Training
Once the environment is launched, initiate training by running:

bash
Copy code
ros2 run d2dtracker_rl rl_node
Acknowledgments
This work is part of the iHunter project under the Robotics & Internet of Things Lab (RIOTU) at Prince Sultan University.

