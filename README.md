# LV-DOT: LiDAR-Visual Dynamic Obstacle Detection and Tracking for Autonomous Robots

This repository implements the LiDAR-visual Dynamic Obstacle Detection and Tracking (LV-DOT) algorithm which aims at detecting and tracking dynamic obstacles for robots with extremely constraint computational resources.

<table>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/4f4df8ac-0b29-4fe7-9e58-8a181c65c7c0" alt="corridor - gif" style="width: 100%;"></td>
    <td><img src="https://github.com/user-attachments/assets/873b1569-f99c-4037-92ad-63e3c6e18e8d" alt="intersection - gif" style="width: 100%;"></td>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/3c5d81a3-7a65-4c1b-9893-90b2d01b760a" alt="office - gif" style="width: 100%;"></td>
    <td><img src="https://github.com/user-attachments/assets/6cad27c8-571c-4e2b-bdce-c4706f61fd36" alt="workspace - gif" style="width: 100%;"></td>
  </tr>
</table>


For additional details, please refer to the related paper available here:


Zhefan Xu\*, Haoyu Shen\*, Xinming Han, Hanyu Jin, Kanlong Ye, and Kenji Shimada, "LV-DOT: LiDAR-visual dynamic obstacle detection and tracking for autonomous robot navigation”, arXiv, 2025. [preprint][\[video\]](https://youtu.be/rRvgTulWqvk)

*The authors contributed equally.


## News
- The GitHub code, video demos, and relavant papers for our LV-DOT framework are released. The authors will actively maintain and update this repo!

## Table of Contents
- [Installation Guide](#Installation-Guide)
- [Run Demo](#Run-Demo)
    - [Run on dataset](#a-Run-on-dataset)
    - [Run on your device](#b-Run-on-your-device)
- [System Framework and Results](#System-Framework-and-Results)
- [Citation and Reference](#Citation-and-Reference)


## Installation Guide
The system requirements for this repository are as follows. Please ensure your system meets these requirements:
- Ubuntu 18.04/20.04 LTS
- ROS Melodic/Noetic

This package has been tested on the following onboard computer:
- [NVIDIA Jetson Xavier NX](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-xavier-series/)
- [NVIDIA Jetson Orin NX](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/) 
- [Intel NUC](https://www.intel.com/content/www/us/en/products/details/nuc.html)
 

Please follow the instructions below to install this package.
```
# This package needs ROS vision_msgs package
sudo apt install ros-noetic-vision-msgs

# Install YOLOv11 required package
pip install ultralytics

cd ~/catkin_ws/src
git clone https://github.com/Zhefan-Xu/LV-DOT.git
cd ..
catkin_make
```


## Run Demo
### a. Run on dataset
Please download the rosbag file from this [link](https://cmu.box.com/s/cucvje5b9xfpdpe57ilh0jx702b3ks2p):
```
rosbag play -l corridor_demo.bag
roslaunch onboard_detector run_detector.launch
```
The perception results can be visualized in Rviz as follows:

https://github.com/user-attachments/assets/e640edab-d4f3-40d6-88dc-9e5014430732


### b. Run on your device
Please adjust the configuration file under ```cfg/detector_param.yaml``` of your LiDAR and camera device. Also, change the color image topic name in ```scripts/yolo_detector/yolov11_detector.py```

From the parameter file, you can find that the algorithm expects the following data from the robot:
- LiDAR Point Cloud: ```/pointcloud```

- Depth image: ```/camera/depth/image_rect_raw```

- Color image: ```/camera/color/image_rect_raw```

- Robot pose: ```/mavros/local_position/pose```

- Robot odometry (alternative to robot pose): ```/mavros/local_position/odom```

Additionally, update the camera intrinsic parameters and the camera-LiDAR extrinsic parameters in the config file.

Run the following command to launch dynamic obstacle detection and tracking.
```
# Launch your sensor device first. Make sure it has the above data.
roslaunch onboard_detector run_detector.launch
```

The LV-DOT can be directly utilized to assist mobile robot navigation and collision avoidance in dynamic environments, as demonstrated below:

<table>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/91a7b945-638a-42ba-9a93-ca1a9b380cab" alt="block - gif" style="width: 100%;"></td>
    <td><img src="https://github.com/user-attachments/assets/cd728204-ad11-44d0-a95d-2286bb3809bc" alt="approach - gif" style="width: 100%;"></td>
  </tr>
</table>

## System Framework and Results
The LV-DOT framework is shown below. Using onboard LiDAR, camera, and odometry inputs, the LiDAR and depth detection modules detect 3D obstacles, while the color detection module identifies 2D dynamic obstacles in the image plane. The LiDAR-visual fusion module refines these detections, and the tracking module classifies obstacles as static or dynamic.

<p align="center">
  <img src="https://github.com/user-attachments/assets/5352c7ae-341a-45c0-8ee1-253d9aed6078" width="90%">
</p>

Example qualitative perception results in various testing environments are shown below:
<p align="center">
  <img src="https://github.com/user-attachments/assets/054e3285-4c44-49e3-939d-74176c5d676e" width="90%">
</p>


## Citation and Reference
If our work is useful to your research, please consider citing our paper.
```
TODO
```

