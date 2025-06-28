# SO-ARM101 ROS2 Bridge

ROS2 workspace and Isaac Sim integration for SO-ARM101 robot hardware interface.

## Complete Tutorial

For detailed step-by-step instructions including LeRobot setup, Isaac Sim configuration, and teleoperation, follow this comprehensive tutorial:

**[SO-ARM101 x Isaac Sim x Isaac Lab Tutorial](https://lycheeai-hub.com/project-so-arm101-x-isaac-sim-x-isaac-lab-tutorial-series/so-arm-teleoperate-real-isaac-sim)**

## Usage with LeRobot

This bridge is designed to work with [LeRobot](https://github.com/huggingface/lerobot) for teleoperation and data collection with SO-ARM101 robots.

First, you need to install LeRobot separately by following their installation instructions and activating the lerobot conda environment.

## Contents

### ROS2 Workspace (`src/`)
- **`jointstatereader`**: Hardware interface package for SO-ARM101 robot
  - Reads joint states from physical SO-ARM101 robot via USB/serial
  - Publishes to `/joint_states` topic
  - Provides topic relay functionality for Isaac Lab integration

### Isaac Sim Integration (`IsaacSim_USD/`)
- **`SO-ARM101-USD.usd`**: Pre-configured Isaac Sim stage file for SO-ARM101 robot
  - Ready-to-use USD stage with SO-ARM101 robot model
  - ROS2 action graph setup for real-time communication

## Quick Start

```bash
# Within the Lerobot Repository:
cd lerobot

# Clone this bridge repository
git clone https://github.com/MuammerBay/so-arm101-ros2-bridge.git
cd so-arm101-ros2-bridge

# Build ROS2 workspace
colcon build --packages-select jointstatereader

# Source workspace
source install/setup.bash

# Run hardware interface (with SO-ARM101 connected)
python3 src/jointstatereader/jointstatereader/joint_state_reader.py
```

## Requirements

- ROS2 Humble
- SO-ARM101 robot hardware
- Isaac Sim (for simulation integration)
- [LeRobot](https://github.com/huggingface/lerobot) (installed separately) 