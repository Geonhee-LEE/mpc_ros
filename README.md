![GitHub issues open](https://img.shields.io/github/issues/Geonhee-LEE/mpc_ros)
![GitHub forks](https://img.shields.io/github/forks/Geonhee-LEE/mpc_ros)
![GitHub stars](https://img.shields.io/github/stars/Geonhee-LEE/mpc_ros)
![GitHub license](https://img.shields.io/github/license/Geonhee-LEE/mpc_ros)


# Nonlinear Model Predictive Control 



## Abstract

The goal of this repository is to implement the path tracking algorithm of mobile robot using the Nonlinear Model Predictive Control(NMPC), one of the optimal controllers. 
The NMPC can provide the poweful and effective performance among existing optimal controllers since it can have the preview ability and easy to regulate constraints. This repository is based on ROS(Robot Operating System). Therefore it supports the local planner plugin and standalone node as well.  


## Features
* Nonlinear Model Pridictive Control (through [ipopt solver](https://coin-or.github.io/Ipopt/))  
* Wheeled Mobile Robot 


### Installation
1. Ubuntu 18.04
2. Install [ROS](http://wiki.ros.org/) Melodic.
3. Install ROS dependencies.

    ```
    sudo apt install ros-melodic-costmap-2d \
    ros-melodic-move-base \
    ros-melodic-global-planner \
    ros-melodic-amcl
    ```
  
4. Install Ipopt: Please refer the tutorial in ["document/ipopt_install"](https://github.com/Geonhee-LEE/mpc_ros/tree/melodic/assets/document/ipopt_install).  
5. Create your own catkin_ws and clone the repositories.
    ```
    git clone https://github.com/Geonhee-LEE/mpc_ros.git 
    ```
    - _(optional)_ If you already have the urdf model, you don't need to clone below  
      ```
      git clone https://github.com/CzJaewan/servingbot.git
      ```
      
    > NOTE: you can also refer other models such as ackermann model, holonomic model. you can see it [mpc_ros_description](https://github.com/Geonhee-LEE/mpc_ros_description)
  
6. Build (_catkin_make_) and Try it.


## Launch

### Run Navigation algorithm with MPC in simulation: 

- It can be selected with DWA, MPC, Pure persuit according to 'controller' argument.
```
roslaunch mpc_ros nav_gazebo.launch
```


### Run tracking the reference line with MPC

- Tracking the trajectory such as infinity-shaped, epitrochoid, square using non-linear model predictive control.
```
roslaunch mpc_ros ref_trajectory_tracking_gazebo.launch
```


## How to use as local planner

- After building successfully, you should just change the base_local_planner param with `mpc_ros/MPCPlannerROS`.
```
<param name="base_local_planner" value="mpc_ros/MPCPlannerROS"/>
```



## Youtube video
---
[![Video Label](http://img.youtube.com/vi/5IqFGBmDGjU/0.jpg)](https://www.youtube.com/watch?v=5IqFGBmDGjU) 


### About us
Contact: gunhee6392@gmail.com  
Date: 2020/05/02  
License: Apache 2.0


### Reference

- HyphaROS MPC MiniCar(https://hypharosworkshop.wordpress.com/)
- Udacity Self-Driving Car Nanodegree - Model Predictive Control (MPC) Project(https://github.com/darienmt/CarND-MPC-Project-P5)
- (Korean) Path Tracking with Nonlinear Model Predictive Control for Differential Drive Wheeled Robot(https://www.dbpia.co.kr/Journal/articleDetail?nodeId=NODE10475067)
