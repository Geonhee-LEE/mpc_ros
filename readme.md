# Model Predictive Control implementation about ROS 

[![Video Label](https://i9.ytimg.com/vi/5IqFGBmDGjU/mqdefault.jpg?sqp=CLCmhuYF&rs=AOn4CLCB8vfISntxgjR0nb9H3YGEjjydMw&time=1556190116601)](https://www.youtube.com/watch?v=5IqFGBmDGjU)



## Abstract
I Strongly refered to the HyphaROS MPC MiniCar(https://hypharosworkshop.wordpress.com/) (Thanks for great source).

I changed the mobile robot model from bycycle to the unicycle which means differential drive wheeled mobile robot for implementing service mobile robot.

Construct the simulation env and compare with DWA algorithm


## About us
Contact: gunhee6392@gmail.com  
Date: 2019/02/16  
License: Apache 2.0 (from HyphaROS MPC MiniCar) 

## Features
* Nonlinear Bicycle Model Based MPC (through ipopt solver)  
* AMCL localization (encoder-odometry based)  
* GAZEBO Simulation (supports: MPC)  

### Install from source (16.04) 
1. Install ROS Kinetic (Desktop-Full) (http://wiki.ros.org/kinetic/Installation/Ubuntu)  
2. Install dependencies:  
```
sudo apt-get install remmina synaptic gimp git ros-kinetic-navigation* ros-kinetic-gmapping ros-kinetic-hector-slam ros-kinetic-mrpt-icp-slam-2d ros-kinetic-slam-karto ros-kinetic-ackermann-msgs -y  
```
3. Install Ipopt: Please refer the tutorial in "document/ipopt_install".  
4. create your own catkin_ws   
(http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace)  
```
cd catkin_ws/src  
git clone https://github.com/Geonhee-LEE/mpc_ros.git
git clone https://github.com/Geonhee-LEE/zetabot-ros.git (private)
cd ..  
catkin_make  
```

## Operation

### Total navigation in simulation: 
```
roslaunch mpc_ros mpc_Gazebo.launch 
```
In the GAZEBO simulation, you can check the local planner as non-linear model predictive control. 
  
  

### NMPC(local planner):
```
roslaunch mpc_ros local_mpc_Gazebo.launch 
```
Tracking the trajectory such as infinity-shaped, epitrochoid, square using non-linear model predictive control.


### DWA:
```
roslaunch mpc_ros dwa_Gazebo.launch 
```
Tracking the trajectory such as infinity-shaped, epitrochoid, square using dynamic window approach.
