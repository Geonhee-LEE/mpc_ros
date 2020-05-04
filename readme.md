# Model Predictive Control implementation about ROS 



## Abstract

This repository is implemented with mobile robot model from bycycle to the unicycle which means differential drive wheeled mobile robot for promising tracking performance. For running this NMPC algorithm, you can use the GAZEBO simulatior or customized mobile robot and compare with DWA algorithm given defalut local planner in ROS. 


## Features
* Nonlinear Unicycle Model Based MPC (through ipopt solver)  
* AMCL localization, fake localization (encoder-odometry based)  
* GAZEBO simulation, [servingbot](https://github.com/NSCL/servingbot_ros)

### Installation
1. Ubuntu 18.04
2. Install ROS Melodic 
3. Install ROS dependencies: 
```
sudo apt install ros-melodic-costmap-2d  ros-melodic-move-base ros-melodic-global-planner ros-melodic-amcl
```
4. Install Ipopt: Please refer the tutorial in "document/ipopt_install".  
5. create your own catkin_ws and clone the repositories. 
```
git clone https://github.com/Geonhee-LEE/mpc_ros.git 
git clone https://github.com/NSCL/servingbot_ros
```

## Launch

### Run Navigation algorithm with MPC in simulation: 
```
roslaunch mpc_ros nav_gazebo.launch
```
It can be selected with DWA, MPC, Pure persuit according to 'controller' argument.


### Run tracking the reference line with MPC
```
roslaunch mpc_ros ref_trajectory_tracking_gazebo.launch
```

Tracking the trajectory such as infinity-shaped, epitrochoid, square using non-linear model predictive control.


## Youtube video
---
[![Video Label](http://img.youtube.com/vi/5IqFGBmDGjU/0.jpg)](https://www.youtube.com/watch?v=5IqFGBmDGjU) mpc


### About us
Contact: gunhee6392@gmail.com  
Date: 2020/05/02  
License: Apache 2.0


### Reference

HyphaROS MPC MiniCar(https://hypharosworkshop.wordpress.com/)

