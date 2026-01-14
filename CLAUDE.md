# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS2 (Humble) robotics project that integrates Intel RealSense D455 depth cameras with YOLOv11 object detection to enable a TurtleBot3 robot to detect and follow people. The system combines real-time depth sensing with computer vision to navigate autonomously.

## Core Architecture

### Three-Program Structure

1. **[turtlebot3_move_forward.py](turtlebot3_move_forward.py)** - Robot connectivity test
   - Tests ROS2 communication with the robot via `/cmd_vel` topic
   - Uses odometry feedback from `/odom` to track distance traveled
   - Fallback to time-based movement if odometry unavailable

2. **[realsense_with_yolo11.py](realsense_with_yolo11.py)** - Camera and detection test
   - Standalone script to verify RealSense camera connectivity
   - Tests YOLO object detection on camera feed
   - Calculates object distance using depth data and heading angle using horizontal FOV
   - Uses spatial and temporal filters to improve depth data quality

3. **[follow_person_node.py](follow_person_node.py)** - Main robot application
   - ROS2 node that combines all functionality
   - Detects closest person using YOLO, computes velocity commands, and publishes to `/cmd_vel`
   - Publishes annotated images to `/detected_object/image` topic
   - Refactored into private member functions for clarity (recent refactor on `code-refactor` branch)

### Key Design Patterns

**RealSense Pipeline Setup**: All scripts use a consistent pattern:
- Configure both color (BGR8) and depth (Z16) streams at 640x480@30fps
- Depth scale is 0.001 (1mm to meters conversion)
- Horizontal FOV is 87° for D455

**Distance Calculation**: Uses median depth from bounding box region to filter noise and handle occlusions robustly.

**Robot Control Logic** (follow_person_node.py):
- If person not centered (>50px offset): rotate towards person
- If person centered but far (>1m): move forward
- If person centered and close (≤1m): stop
- If no person detected: slow rotation to search

## Environment Setup

### ROS2 Configuration
- ROS Distribution: `humble`
- Default ROS_DOMAIN_ID: `55`
- Default TURTLEBOT3_MODEL: `burger`
- RMW Implementation: `rmw_cyclonedds_cpp`

These are set in devcontainer environment and in script headers.

### Development Container
The project uses a devcontainer with Ubuntu 22.04 and includes:
- ROS2 Humble (ros-base + required message packages)
- Intel RealSense SDK (librealsense2-dev)
- CPU-only PyTorch for development (no GPU required)
- Python packages: pyrealsense2, ultralytics, opencv-python, numpy

The devcontainer provides LSP support but **does not have camera hardware access**. Physical testing requires deployment to actual hardware.

## Running the Programs

### Prerequisites on Target Hardware
- RealSense D455 camera connected
- TurtleBot3 robot accessible on network (if testing robot control)
- YOLO model file `yolo11s.pt` in workspace root

### Test Camera and Detection
```bash
python3 realsense_with_yolo11.py
```
Opens a GUI window showing detected objects with distance and heading angle.

### Test Robot Connectivity
```bash
python3 turtlebot3_move_forward.py
```
Prompts for TurtleBot3 model, then moves robot forward 1 meter using odometry feedback.

### Run Main Application
```bash
python3 follow_person_node.py
```
Starts the ROS2 node that makes the robot follow the closest detected person.

## Hardware Deployment Notes

### For Nvidia Jetson Hosts
- **Critical**: Must use JetPack version compatible with librealsense (currently 6.0 for Orin)
- Build librealsense from source using native backend installation
- Use specific PyTorch/torchvision wheels for ARM64 (see README for correct versions)
- Install with `uv pip install` instead of `pip` (ultralytics[export] fails with pip on Jetson)

### For x86/AMD64 Hosts
- Can install pre-built librealsense packages via apt
- Standard pip/uv installation works for all Python packages
- Much simpler setup than Jetson

## Code Structure Conventions

- Scripts set environment variables (`ROS_DOMAIN_ID`, `TURTLEBOT3_MODEL`) at module level before importing ROS libraries
- Recent refactoring (commit 7417d30) moved inline logic into private member functions with leading underscore (e.g., `_calculate_distance`, `_find_closest_person`, `_compute_velocity_command`, `_annotate_image`)
- Image dimensions and camera parameters defined as class constants (uppercase)
- All scripts handle cleanup (pipeline.stop(), cv2.destroyAllWindows()) in finally blocks or destroy_node()

## Important Technical Details

**Depth Data Format**: RealSense depth stream uses Z16 format (raw millimeters). Always multiply by depth_scale (0.001) to convert to meters.

**YOLO Model Access**: Model class names accessed via `model.names[int(class_id)]`. Bounding boxes returned as tensors requiring `.cpu().numpy()` conversion.

**ROS2 Message Flow**:
- Input: `/odom` (Odometry) for position tracking
- Output: `/cmd_vel` (Twist) for velocity commands
- Output: `/detected_object/image` (Image) for annotated camera view

**Control Loop Frequency**: follow_person_node runs at 10Hz (0.1s timer) with 100ms timeout on frame acquisition.
