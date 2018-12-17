# HyphaROS MiniCar (1/20 Scale MPC Racing Car)
![alt text](https://github.com/Hypha-ROS/hypharos_minicar/blob/master/document/photo/HyphaROS_logo.png)  

## Abstract
Low cost, High speed 1/20 Racing Car for control laws evaluation !   
Fully open-sourced (hardware & software), total cost <300USD.  
Currently supports: Pure-Pursuit, Model-Predictive-Control (Nonlinear)  

![alt text](https://github.com/Hypha-ROS/hypharos_minicar/blob/master/document/photo/HyphaROS_MiniCar_photo.jpg)  

## About us
FB Page: https://www.facebook.com/HyphaROS/  
Website: https://hypharosworkshop.wordpress.com/  
Contact: hypha.ros@gmail.com  
Developer:   
* HaoChih, LIN  

Date: 2018/08/16  
License: Apache 2.0  

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

## Hardware 
* Odroid XU4  
(Ref: https://www.hardkernel.com/main/products/prdt_info.php)   
* YDLidar X4  
(Ref: http://www.ydlidar.com/product/X4)  
* STM32(F103) MCU (with OLED display)  
* Diff Motor with A/B encoder(res: 340)  
* Ackermann Based 1/20 car chassis  
(Ref: https://item.taobao.com/item.htm?spm=a312a.7700824.w4004-15726392027.68.79503c88Rqwzb9&id=554619475840)   
Total Cost: < 300 USD  

## Document  
HyphaROS MPC MiniCar 1-Day Workshop:  
https://drive.google.com/open?id=1yX0aeA4spf_szpxXFpIlH0EQKIgiwJx7  
ROS Summer School in China 2018 Slides:  
https://goo.gl/RpVBDH  

## Software
### VirtualBox Image ###  
OVA image file (Kinetic, password: hypharos, 20180816)  
Link: https://drive.google.com/open?id=1uRvXGakvQrbynmPHX_KIFJxPm8o6MWPb  

### Odroid Image
Image file for Odroid XU4.(with SD card >=16G, password: hypharos)  
Link: https://goo.gl/87vrNk   
(if your SD card is around 13GB, it's OK to force Win32DiskImager to write the file!)   
The default ethernet IP address is 10.0.0.1  

### STM32 (MCU)
Source codes: https://drive.google.com/open?id=19rjjpJmz85lTSxCyu-9CtvZhUW37c2LS         
[WARNING!]  
Since the original STM32 codes came from third-paries,   
currently, the quality of codes are not guaranteed by us.  
For MCUISP Driver: http://www.mcuisp.com/English%20mcuisp%20web/ruanjianxiazai-english.htm  

### Install from source (16.04) 
1. Install ROS Kinetic (Desktop-Full) (http://wiki.ros.org/kinetic/Installation/Ubuntu)  
2. Install dependencies:  
$ sudo apt-get install remmina synaptic gimp git ros-kinetic-navigation* ros-kinetic-gmapping ros-kinetic-hector-slam ros-kinetic-mrpt-icp-slam-2d ros-kinetic-slam-karto ros-kinetic-ackermann-msgs -y  
3. Install Ipopt: Please refer the tutorial in "document/ipopt_install".  
4. create your own catkin_ws   
(http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace)  
$ cd catkin_ws/src  
$ git clone https://github.com/EAIBOT/ydlidar  
$ git clone https://github.com/Hypha-ROS/hypharos_minicar   
$ cd ..  
$ catkin_make  

## Operation
### Simulation
$ roslaunch hypharos_minicar HyphaROS_Simulation_Stage.launch  
The default controller is mpc, you can switch to pure-pursuit or DWA through param: "controller"    
![alt text](https://github.com/Hypha-ROS/hypharos_minicar/blob/master/document/photo/HyphaROS_MPC_MiniCar_Simulation.jpg)    
  
### Ethernet Connection
The default static eth IP on Odroid image is 10.0.0.1, hence, to connect to your Odroid through cable, please set your host IP as 10.0.0.X  
Notice: for the first bootup, you have to update Odroid MAC address through HDMI Dispaly!  

### Wifi Connection
Use ethernet or display connection to make Odroid connect to your local Wifi AP. Remember to set ROS_MASTER_URI and ROS_IP in ".bashrc" file on both Odroid & host machine.    

### Mapping
* MiniCar (Odroid) side:  
$ roslaunch hypharos_minicar HyphaROS_MiniCar_Mapping.launch  
The default mapping algorithm is gmapping, you can swith to other slam method through param: "slam_type"  
(crrently supports: gmapping, karto_slam, mrpt_icp and hector_slam)  
  
* Host (Desktop) side:  
$ roslaunch hypharos_minicar HyphaROS_Desktop_Mapping.launch  
Use keyboard to control the minicar.  
  
After mapping, remember to save two maps, one for amcl and the other for move_base!  

### Racing
* MiniCar (Odroid) side:  
$ roslaunch hypharos_minicar HyphaROS_MiniCar_Racing.launch  
The default controller is mpc, you can swith to other slam method through param: "controller"  
(crrently supports: mpc and pure_pursuit)  
  
* Host (Desktop) side:  
$ roslaunch hypharos_minicar HyphaROS_Desktop_Racing.launch  
Use keyboard to interrupt controller's behavior.  

