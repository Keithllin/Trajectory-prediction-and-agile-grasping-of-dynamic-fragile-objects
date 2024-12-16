# Trajectory Prediction and Agile Grasping of Dynamic Fragile Objects

## Table of Contents
- Overview
- Features
- Project Structure
- Prerequisites
- Installation
- Usage
  - 1. Launching the ROS Core
  - 2. Starting the UR5 Manipulator
  - 3. Running the YOLO Detection Node
  - 4. Running the Trajectory Publisher
  - 5. Running the LSTM Trajectory Prediction Node
  - 6. Running the UR5 Manipulator Control Node
- Model Training
- Configuration


## Overview

This project integrates computer vision, deep learning, and robotic motion control to predict the trajectory of dynamic fragile objects and autonomously grasp them using a UR5 robotic arm. The system leverages a YOLO-based detection module for object identification, an LSTM neural network for trajectory prediction, and MoveIt for motion planning and execution.

## Features

- **Real-time Object Detection:** Utilizes YOLO for fast and accurate detection of target objects (e.g., eggs) in the camera feed.
- **Trajectory Prediction:** Employs an LSTM model to predict the future positions of detected objects based on their motion history.
- **Autonomous Grasping:** Controls a UR5 robotic arm to navigate to the predicted positions and perform precise grasping actions.
- **Data Visualization:** Provides visualization of detected objects and their trajectories for monitoring and debugging purposes.
- **Robust Integration:** Seamlessly integrates various ROS nodes to ensure synchronized operations between detection, prediction, and manipulation.

## Project Structure

```
Trajectory-Prediction-and-Grasping/
├── README.md
├── src/
│   └── realsense_example/
│       └── scripts/
│           ├── 01_YOLO_OD.py
│           ├── 02_trajetory_pub.py
│           ├── 03_lstm_trajectory_sub.py
│           ├── 04_ur5_mani.py
│           └── lstmmodel.py
├── detection/
│   ├── train_mymodel.py
│   ├── make_dataset.py

```

- **`scripts/`**: Contains all ROS node scripts.
  - **`01_YOLO_OD.py`**: YOLO-based object detection node.
  - **`02_trajetory_pub.py`**: Publishes object trajectory data.
  - **`03_lstm_trajectory_sub.py`**: Subscribes to trajectory data, performs prediction using LSTM, and publishes predicted positions.
  - **`04_ur5_mani.py`**: Subscribes to predicted positions and controls the UR5 manipulator for grasping.
  - **`lstmmodel.py`**: Defines the LSTM model structure used for trajectory prediction.
- **`detection`**: Contains scripts and files related to training the YOLOv8.
  - **`train_mymodel.py`**: Script for training the YOLOv8 model.
  - **`make_dataset.py`**: make your own dataset
  

## Prerequisites

- **Operating System:** Ubuntu 20.04
- **ROS Version:** ROS Noetic
- **Python Version:** Python 3.8+
- **Hardware:** UR5 Robotic Arm, Intel RealSense Camera
- **Dependencies:**
  - [PyTorch](https://pytorch.org/)
  - [OpenCV](https://opencv.org/)
  - [Ultralytics YOLO](https://github.com/ultralytics/yolov5)
  - [MoveIt](https://moveit.ros.org/)
  - [ROS Packages](https://wiki.ros.org/Packages):
    - 

sensor_msgs


    - 

std_msgs


    - `geometry_msgs`
    - `moveit_commander`
    - 

cv_bridge


    - `tf2_ros`
    - `dh_gripper_msgs` (Custom or third-party package for gripper control)

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/Keithllin/Trajectory-prediction-and-agile-grasping-of-dynamic-fragile-objects
cd Trajectory-Prediction-and-Grasping
```

### 2. Set Up ROS Workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
ln -s ~/Trajectory-Prediction-and-Grasping/src/realsense_example .
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 3. Install Required Dependencies

```bash
# Update package lists
sudo apt-get update

# Install ROS dependencies
sudo apt-get install ros-noetic-moveit ros-noetic-ur5-driver python3-pip

# Install Python libraries
pip3 install torch torchvision numpy scipy scikit-learn opencv-python ultralytics pyrealsense2
```

### 4. Configure the UR5 and RealSense Camera

- **UR5 Setup:**
  - Ensure the UR5 robotic arm is properly connected and the drivers are installed.
  - Verify communication with the UR5 by running example MoveIt nodes.

- **RealSense Camera Setup:**
  - Connect the Intel RealSense camera.
  - Install RealSense SDK if not already installed.
  - Verify camera feed using RealSense viewer.

## Usage

### 1. Launch the ROS Core

```bash
roslaunch ur_robot_driver ur5_bringup.launch limited:=true robot_ip:= your_own_ip
```

### 2. Launch the UR5 Manipulator

```bash
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch
```

### 3. Run the YOLO Detection Node

```bash
rosrun realsense_example 01_YOLO_OD.py
```

### 4. Run the Trajectory Publisher

```bash
rosrun realsense_example 02_trajetory_pub.py
```

### 5. Run the LSTM Trajectory Prediction Node

```bash
rosrun realsense_example 03_lstm_trajectory_sub.py
```

### 6. Run the UR5 Manipulator Control Node

```bash
rosrun realsense_example 04_ur5_mani.py
```



## Model Training

To train or retrain the LSTM model for trajectory prediction, follow these steps:

### 1. Navigate to the LSTM Training Directory

```bash
cd ~/Trajectory-Prediction-and-Grasping/scripts
```

### 2. Prepare the Dataset

Ensure your trajectory data is in `.npy` format and placed in the `trajectories/` directory. Update the `lstmmodel.py` script with the correct file paths if necessary.

### 3. Train the Model

```bash
python3 lstmmodel.py
```

This script will:

- Load and preprocess the trajectory data.
- Train the LSTM model.
- Save the trained model weights as 

lstm_model.pth

.
- Save the scaler object for data normalization as 

scaler.pkl

.

### 4. Update Model and Scaler Paths

After training, ensure that the `03_lstm_trajectory_sub.py` script points to the newly trained 

lstm_model.pth

 and 

scaler.pkl

 files.

## Configuration

### 1. Update File Paths

Ensure all file paths in the scripts point to their respective files correctly. Modify paths in the following scripts as needed:

- **`01_YOLO_OD.py`**
  - YOLO model path: 

model_path = "/path/to/best.pt"



- **`03_lstm_trajectory_sub.py`**
  - LSTM model path: 

model_path = "/path/to/lstm_model.pth"


  - Scaler path: `scaler_path = "/path/to/scaler.pkl"`

### 2. Adjust Buffer Sizes and Thresholds

Modify buffer sizes, depth thresholds, and other parameters in the scripts to suit your specific application and hardware setup.

### 3. Customize Transformation Matrices

Ensure that the transformation matrices used for converting camera coordinates to base coordinates are calibrated correctly.



