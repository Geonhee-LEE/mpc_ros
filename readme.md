# Model Predictive Control implementation about ROS 


## Abstract
Strongly refered on the HyphaROS MPC MiniCar(https://hypharosworkshop.wordpress.com/ ).
I change the mobile robot model from bycycle to the unicycle which means differential drive wheeled mobile robot for implementing service mobile robot.


## About us
Contact: gunhee6392@gmail.com  
Date: 2019/02/16  
License: Apache 2.0 (from HyphaROS MPC MiniCar) 

## Features
* Nonlinear Bicycle Model Based MPC (through ipopt solver)  
* Pure-Pursuit Controller  
* Onboard mapping (Gmapping, Hector-SLAM, Karto-SLAM, MRPT-ICP)  
* STM32 for motor speed closed-loop control  
* AMCL localization (encoder-odometry based)  
* Dynamic obstacle avoidance  
* Stage Simulation (supports: MPC & Pure-Pursuit)  

## Roadmap
* Add EKF supports (odometry with mpu6050)  
* MPC for obstacle avoidance  
* Implement MPC on different solvers (ACADO, OSQP, etc)  
* Multi-cars racing through ROS 2.0 layer  
* High Speed Drifting  


### Install from source (16.04) 
1. Install ROS Kinetic (Desktop-Full) (http://wiki.ros.org/kinetic/Installation/Ubuntu)  
2. Install dependencies:  
$ sudo apt-get install remmina synaptic gimp git ros-kinetic-navigation* ros-kinetic-gmapping ros-kinetic-hector-slam ros-kinetic-mrpt-icp-slam-2d ros-kinetic-slam-karto ros-kinetic-ackermann-msgs -y  
3. Install Ipopt: Please refer the tutorial in "document/ipopt_install".  
4. create your own catkin_ws   
(http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace)  
$ cd catkin_ws/src  
$ git clone https://github.com/EAIBOT/ydlidar  
$ git clone https://github.com/Hypha-ROS/mpc_ros   
$ cd ..  
$ catkin_make  

## Operation
### Simulation
$ roslaunch mpc_ros HyphaROS_Simulation_Stage.launch  
The default controller is mpc, you can switch to pure-pursuit or DWA through param: "controller"     
  
### Ethernet Connection
The default static eth IP on Odroid image is 10.0.0.1, hence, to connect to your Odroid through cable, please set your host IP as 10.0.0.X  
Notice: for the first bootup, you have to update Odroid MAC address through HDMI Dispaly!  

### Wifi Connection
Use ethernet or display connection to make Odroid connect to your local Wifi AP. Remember to set ROS_MASTER_URI and ROS_IP in ".bashrc" file on both Odroid & host machine.    

### Mapping
* MiniCar (Odroid) side:  
$ roslaunch mpc_ros mpc_ros_Mapping.launch  
The default mapping algorithm is gmapping, you can swith to other slam method through param: "slam_type"  
(crrently supports: gmapping, karto_slam, mrpt_icp and hector_slam)  
  
* Host (Desktop) side:  
$ roslaunch mpc_ros HyphaROS_Desktop_Mapping.launch  
Use keyboard to control the minicar.  
  
After mapping, remember to save two maps, one for amcl and the other for move_base!  

### Racing
* MiniCar (Odroid) side:  
$ roslaunch mpc_ros mpc_ros_Racing.launch  
The default controller is mpc, you can swith to other slam method through param: "controller"  
(crrently supports: mpc and pure_pursuit)  
  
* Host (Desktop) side:  
$ roslaunch mpc_ros HyphaROS_Desktop_Racing.launch  
Use keyboard to interrupt controller's behavior.  


