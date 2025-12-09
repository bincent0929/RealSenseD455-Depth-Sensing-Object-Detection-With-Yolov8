Before you do this, install `uv` so that your `pip` installs don't get SUPER hung up.

`pip install uv`

Basically a superset that's WAY more efficient than `pip`. 

All you have to do to use it is place it in front of your `pip` commands.

Like this:
```
uv pip install cool-package
```
***
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
Heads up, the guide has you install torch and torchvision that don't support Jetpack 6.0!

Use these instead when you reach that step:
[correct torch wheel from ultralytics](https://github.com/ultralytics/assets/releases/download/v0.0.0/torch-2.3.0-cp310-cp310-linux_aarch64.whl)
[correct torchvision wheel from ultralytics](https://github.com/ultralytics/assets/releases/download/v0.0.0/torchvision-0.18.0a0+6043bc2-cp310-cp310-linux_aarch64.whl)
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
***
Now, to get started, you need to follow [this](https://docs.ultralytics.com/guides/nvidia-jetson/#install-ultralytics-package_1) to make it work WAY better than just plain doing a `pip install`!

# RealSense Depth Sensing Object Detection with YOLOv5

This repository aims to integrate the RealSense D455 Depth Sensing Camera with the YOLOv5 object detection algorithm for enhanced object detection accuracy and performance. By incorporating depth information, the project strives to improve object localization and recognition in real-world environments.

## Features

- Integration of the RealSense D455 Depth Sensing Camera with YOLOv5
- Utilizes depth information for more precise object localization
- Improved accuracy and robustness in object detection
- Real-time object detection and depth visualization

## Prerequisites

- Python 3.x
- Intel RealSense SDK
- PyTorch
- OpenCV
- Other necessary dependencies (listed in requirements.txt)

## Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/Paulraj916/YOLOv5_with_RealSense_D455_Depth_Sensing.git
   ```
   
2. Install the required dependencies:

    ```bash
   pip install -r requirements.txt
    ```
    
3. Run the main script:

    ```bash
   python  depthScale_realsense.py
    ```
## Demo

![Demo of this project](view.gif)
sample video [here](https://youtu.be/FH7up4knf6w)
