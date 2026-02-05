---
sidebar_position: 2
title: "Isaac Sim Dependencies and Setup"
---

# Isaac Sim Dependencies and Setup

## Required Dependencies

### Isaac Sim Core Dependencies
- **NVIDIA GPU**: With CUDA compute capability 6.0+
- **NVIDIA Driver**: Version 470 or later
- **CUDA Toolkit**: Version 11.0 or later
- **OpenGL**: Version 4.5 or later
- **Docker**: For containerized deployment (optional but recommended)
- **Python**: Version 3.8-3.10 for Isaac ROS bridge

### Isaac ROS Dependencies
- **ROS 2 Humble Hawksbill**: Primary ROS 2 distribution
- **Isaac ROS Packages**:
  - `isaac_ros_common`
  - `isaac_ros_image_pipeline`
  - `isaac_ros_vslam`
  - `isaac_ros_perceptor`
  - `isaac_ros_visual_slam`
  - `isaac_ros_point_cloud_interfaces`

### Nav2 Dependencies
- **Navigation2 Packages**:
  - `nav2_bringup`
  - `nav2_core`
  - `nav2_behavior_tree`
  - `nav2_planner`
  - `nav2_controller`
  - `nav2_rviz_plugins`

## Setup Instructions

### 1. System Preparation

```bash
# Update system packages
sudo apt update && sudo apt upgrade

# Install essential build tools
sudo apt install build-essential cmake pkg-config
```

### 2. Install NVIDIA Drivers

```bash
# Check current driver version
nvidia-smi

# If upgrading, install latest driver
sudo apt install nvidia-driver-535
sudo reboot
```

### 3. Install CUDA Toolkit

```bash
# Download and install CUDA
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/7fa2af80.pub
sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/ /"
sudo apt update
sudo apt install cuda-toolkit-11-8
```

### 4. Install ROS 2 Humble Hawksbill

```bash
# Set up locale
locale-gen en_US.UTF-8
export LANG=en_US.UTF-8

# Set up sources
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS 2
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-argcomplete
```

### 5. Install Isaac ROS Packages

```bash
# Source ROS environment
source /opt/ros/humble/setup.bash

# Install Isaac ROS meta-package
sudo apt install ros-humble-isaac-ros-dev

# Install specific Isaac ROS packages
sudo apt install ros-humble-isaac-ros-common
sudo apt install ros-humble-isaac-ros-image-pipeline
sudo apt install ros-humble-isaac-ros-vslam
sudo apt install ros-humble-isaac-ros-perceptor
sudo apt install ros-humble-isaac-ros-visual-slam
```

### 6. Install Navigation2 Packages

```bash
# Install Navigation2 packages
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-gui
```

## Docker Setup (Alternative)

For a containerized approach:

```bash
# Install Docker
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io

# Add user to docker group
sudo usermod -aG docker $USER

# Log out and back in for changes to take effect

# Pull Isaac Sim container
docker pull nvcr.io/nvidia/isaac-sim:latest

# Pull Isaac ROS containers
docker pull nvcr.io/nvidia/isaac_ros:vslam
docker pull nvcr.io/nvidia/isaac_ros:image_pipeline
```

## Environment Setup

### 1. Create ROS Workspace

```bash
# Create workspace
mkdir -p ~/isaac_ws/src
cd ~/isaac_ws

# Source ROS
source /opt/ros/humble/setup.bash

# Build workspace
colcon build --symlink-install
```

### 2. Environment Variables

Add to your `~/.bashrc`:

```bash
# ROS 2 Humble
source /opt/ros/humble/setup.bash

# Isaac workspace
source ~/isaac_ws/install/setup.bash

# CUDA
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
```

## Verification

### 1. Check ROS Installation

```bash
# Verify ROS installation
ros2 --version

# Check for Isaac ROS packages
ros2 pkg list | grep isaac
```

### 2. Check Isaac Sim Dependencies

```bash
# Check CUDA
nvidia-smi
nvcc --version

# Check OpenGL
glxinfo | grep "OpenGL version"
```

### 3. Test Isaac ROS Nodes

```bash
# List available Isaac ROS nodes
ros2 run | grep isaac

# Test basic Isaac ROS functionality
ros2 run isaac_ros_visual_slam visual_slam_node --ros-args -p enable_rectification:=true
```

## Troubleshooting Common Issues

### CUDA Not Found
- Ensure CUDA toolkit is properly installed
- Check that environment variables are set correctly
- Verify NVIDIA drivers are compatible with CUDA version

### Isaac ROS Packages Missing
- Verify ROS 2 humble is installed
- Check that Isaac ROS repository is added
- Ensure internet connection for package downloads

### GPU Access Denied
- Add user to video and render groups: `sudo usermod -a -G video,render $USER`
- Check if secure boot is interfering with NVIDIA drivers
- Verify that no other processes are blocking GPU access

## Next Steps

Once all dependencies are installed and verified, you're ready to explore Isaac Sim fundamentals and begin creating your first photorealistic simulations.