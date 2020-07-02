cmake_minimum_required(VERSION 3.5)
project(mpc_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11 -O3)

# options for build configuration
option(BUILD_EXAMPLE "Whether or not building the CppAD & Ipopt example" OFF) 

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  #ackermann_msgs
  message_generation
  move_base
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
)


catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES mpc_ros
#  CATKIN_DEPENDS costmap_2d dynamic_reconfigure geometry_msgs message_generation move_base roscpp rospy std_msgs tf visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( ${catkin_INCLUDE_DIRS} include )
include_directories(/usr/include)
link_directories(/usr/lib)


# General MPC_Node 
ADD_EXECUTABLE( MPC_Node src/MPC.cpp src/MPC_Node.cpp )
TARGET_LINK_LIBRARIES(MPC_Node ipopt ${catkin_LIBRARIES} )

# Total navigation with MPC_Node
ADD_EXECUTABLE( nav_mpc src/navMPC.cpp src/navMPCNode.cpp )
TARGET_LINK_LIBRARIES(nav_mpc ipopt ${catkin_LIBRARIES} )

# Local planner with MPC_Node for tracking
ADD_EXECUTABLE( tracking_reference_trajectory src/trackRefTraj.cpp src/trackRefTrajNode.cpp )
TARGET_LINK_LIBRARIES(tracking_reference_trajectory ipopt ${catkin_LIBRARIES} )

# Pure Pursuit Node
add_executable(Pure_Pursuit src/Pure_Pursuit.cpp)
target_link_libraries(Pure_Pursuit ${catkin_LIBRARIES})

# Own Global planner for tracking desired trajectory
 add_library(global_planner_lib src/global_planner/global_planner.cpp)
 TARGET_LINK_LIBRARIES(global_planner_lib ipopt ${catkin_LIBRARIES} )

#############
## Example ##
#############
if(BUILD_EXAMPLE)
    ADD_EXECUTABLE( CppAD_started example/CppAD_started.cpp )
    TARGET_LINK_LIBRARIES(CppAD_started)
    
    ADD_EXECUTABLE( CppAD_Ipopt example/CppAD_Ipopt.cpp )
    TARGET_LINK_LIBRARIES(CppAD_Ipopt ipopt)
endif(BUILD_EXAMPLE)
