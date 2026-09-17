# Humanoid MuJoCo

<div align="center">

**MuJoCo simulation for the VitaBoy humanoid — the same ROS 2 topics as the real robot, so [`honor_rl_deploy`](https://github.com/HonorRobotics/honor_rl_deploy) drives it identically.**

[![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![MuJoCo](https://img.shields.io/badge/Physics-MuJoCo-1A73E8)](https://mujoco.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/22.04/)

[English](readme.md) | [简体中文](readme_zh.md)

</div>

## ✨ Overview

This repository simulates the VitaBoy humanoid in MuJoCo and talks to the rest of the stack over ROS 2, so the policy running in the C++ inference repository drives it exactly as it would drive the real robot.

![Overview diagram](docs/overview.svg)

### Features

- **Same ROS 2 interface** — identical topics to the real robot; the C++ controller runs unchanged against sim and hardware.
- **One-command launch** — `run_mujoco.sh` sets up the uv env, sources ROS 2, and starts the viewer (run `build_sdk_msgs.sh` once first for the `honor_robot_sdk` message packages).
- **Elastic band helper** — a virtual spring holds the robot up while you tune a policy.
- **Configurable** — robot, scene path and loop periods in `scripts/config.py`.

## 📦 Setup

### Requirements

- Linux (Ubuntu 22.04)
- [uv](https://docs.astral.sh/uv/getting-started/installation/)
- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### Clone the project

```bash
git clone https://github.com/HonorRobotics/honor_mujoco.git

git clone https://github.com/HonorRobotics/honor_robot_sdk.git
```

### Install dependencies

```bash
cd honor_mujoco

./scripts/install.sh
```

## 🚀 Usage

### Start MuJoCo

```bash
cd honor_mujoco

source ./scripts/build_sdk_msgs.sh --honor-sdk /your_path/to_sdk

./scripts/run_mujoco.sh
```

### Viewer keys

The elastic band is a virtual spring that holds the robot up while you tune a policy. With the viewer focused:


| Key | Effect                                |
| --- | ------------------------------------- |
| `7` | shorten the band, lifting the robot   |
| `8` | lengthen the band, lowering the robot |
| `9` | switch the band off or on             |

> 💡 **Tip:** Set `ENABLE_ELASTIC_BAND = False` in `scripts/config.py` to disable the band entirely.

## 🛠️ Development

```text
<workspace>/
└── honor_mujoco/
    ├── scripts/                # MuJoCo simulation, run from inside this directory
    │   ├── install.sh          # checks uv + ROS, creates the uv env, installs deps
    │   ├── build_sdk_msgs.sh   # build + source honor_robot_sdk/common message packages
    │   ├── run_mujoco.sh       # launcher: uv env + ROS, then run_mujoco.py
    │   ├── run_mujoco.py       # entry point: viewer plus physics and render threads
    │   ├── ros2_bridge.py      # ROS 2 node: joint commands in, state and IMU out
    │   └── config.py           # robot, scene path and loop periods
    └── robots_assets/
        └── vita_boy/vita_boy1.0/     # scene XML and meshes
```
