# webots_iilab
* Webots package to interface with ROS, based on the webots_ros 2023.1.0 package
* This package works both for 
  * Internal webots controllers (compiled and executed inside the webots GUI. These can use ROS functionalities)
## Pre-requisites
### ROS Noetic
### Webots R2023b
* [Link](https://cyberbotics.com/doc/guide/installation-procedure)
  * "Installing the Debian Package with the Advanced Packaging Tool (APT)"
## Installation
* Assuming catkin_ws is already created and configured
* Clone the repository into the "catkin_ws/src/" directory
```bash
git clone https://github.com/JMCOliveira02/webots_iilab.git
```
* Inside the new folder named "webots_iilab", some notes on important folders:
  * Worlds: Webots projects are stored here, each file saves information about the current state of simulation, what controller is being used, what objects are being imported etc.
    * iilab_test.wbt
    * L_scenario.wbt
  * Controllers: Each internal webots controller is stored here
    * ros_interface
  * Objects: Some .obj files that can be imported by the world files
    * iilab.obj
    * Map_test.obj
  * Src: This folder is where the code for ROS nodes should go.
### Python 3.10 environment
* Torch
* Scikit-learn
* opencv

## Usage
### Webots robot
* Run roscore
* Open webots and open world titled "iilab_test.wbt"
* Select the robot in the scene tree and make sure the controller is "ros_interface"
  * It is possible to visualize and configure the ros_interface source code by clicking "Edit"
    * Configuring robot mode of operation (Normal or dataset creator) 
      * Normal: Constantly reads and publishes from and to all open topics
      * Dataset Creator: Waits for a set position command before sending the PointCloud from the 3D Lidar
    * Configuring verbose output
  * Reload the simulation, to restart the controller
* It should now be possible to control the robot and read the lidar scan through the designated topics
  * Input: /cmd_vel to control the velocity
  * Input: /set_position to set the robot's position and orientation
  * Output: /scan2D contains the LaserScan message of a 2D Lidar placed in the robot
  * Output: /scan3D contains the PointCloud2 message of a 3D Lidar placed in the robot
  * Output: /tf contains the robot's Pose message
### Dataset creation
* rosrun webots_iilab pointcloud_saver
* By assigning the corners of a rectangle on the map and step in x and y direction in pointcloud_saver.cpp, the robot will go to each position, take a LiDAR measurment and save both the measurement and pose to designated folders
* In target_position.txt, the coordinates for each rectangular section considered for the dataset "iilab"
### Kd-Tree Pose estimation
* rosrun webots_iilab pose_estimate.py
* iilab.jpg for the 2D mini-map
* poses.npy, descriptors_xxx.npy and xxx.pth are the poses and associated descriptors from the PointNet feature extraction (done previously). The.pth file stores the weights used for the PointNet during feature extraction (must be the same during both stages)
