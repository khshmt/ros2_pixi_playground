# ros2_pixi_playground

A minimal **ROS 2 C++ playground** that runs completely inside a **pixi environment** —  
no system-wide ROS installation required.

The project demonstrates how to:

- Install ROS 2 dependencies using `pixi`
- Build ROS 2 nodes using plain `cmake` (not colcon)
- Publish & subscribe camera frames
- Record data using `rosbag2`
- Work with DDS directly and through rclcpp
- Display camera frames using rerun-sdk

---

## Why this exists

Normally ROS 2 requires a full system installation and colcon workspace.  
This repository shows a different workflow:

> Treat ROS 2 like a normal C++ library dependency.

This makes ROS usable inside:
- isolated environments
- CI pipelines
- research prototypes
- non-ROS projects that only need messaging

---

## Setup

### 1) Install pixi
https://pixi.prefix.dev/latest/installation/

### 2) Create the environment
```bash
pixi install
pixi shell
pixi run build
```

## Run Example
```bash
pixi run publisher # terminal-1
pixi run subscriber # terminal-2
```
### another example with usb web camera
```bash
pixi run webCameraPublisher # terminal-1
pixi run webCameraSubscriber # terminal-2
pixi run webCameraFrameVisualizer # terminal-3
```
