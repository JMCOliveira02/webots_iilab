# webots_iilab
* Webots package to interface with ROS, based on the webots_ros 2023.1.0 package
* This package works both for 
  * Internal webots controllers (compiled and executed inside the webots GUI. These can use ROS functionalities)
  * ROS nodes (compiled using catkin_make and executed externally) 
    * **NOTE** The *ros* controller inside Webots should be running for this
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
  * Controllers: Each internal webots controller is stored here
    * Supervisor
  * Objects: Some .obj files that can be imported by the world files
    * iilab.obj
  * Src: This folder is where the code for ROS nodes should go.
