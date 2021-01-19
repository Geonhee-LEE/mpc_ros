/*
 * mpc_ros
 * Copyright (c) 2021, Geonhee Lee
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

#ifndef MPC_LOCAL_PLANNER_NODE_ROS_H
#define MPC_LOCAL_PLANNER_NODE_ROS_H

#include <vector>
#include <map>
#include <Eigen/Core>
// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>
// local planner specific classes which provide some macros
#include <base_local_planner/goal_functions.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>

#include "ros/ros.h"
#include "MPCLocalPlanner.h"
#include <iostream>
#include <math.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <Eigen/QR>
//#include <ackermann_msgs/AckermannDriveStamped.h>


using namespace std;

namespace mpc_ros{

    class MPCPlannerROS : public nav_core::BaseLocalPlanner
    {

        public:
            MPCPlannerROS();
            ~MPCPlannerROS();
            MPCPlannerROS(std::string name, 
                           tf2_ros::Buffer* tf,
                costmap_2d::Costmap2DROS* costmap_ros);
        
            // Solve the model given an initial state and polynomial coefficients.
            // Return the first actuatotions.
            vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
            vector<double> mpc_x;
            vector<double> mpc_y;
            vector<double> mpc_theta;

            void LoadParams(const std::map<string, double> &params);

            // Local planner plugin functions
            void initialize(std::string name, tf2_ros::Buffer* tf,
                costmap_2d::Costmap2DROS* costmap_ros);
            bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
            bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
            bool isGoalReached();
        private:
            //Pointer to external objects (do NOT delete object)
            costmap_2d::Costmap2DROS* costmap_ros_; ///<@brief pointer to costmap  
            // Flags
            bool goal_reached_;
            bool initialized_;


        public:
            int get_thread_numbers();
        private:
            ros::NodeHandle _nh;
            ros::Subscriber _sub_odom, _sub_path, _sub_goal, _sub_amcl;
            ros::Publisher _pub_globalpath,_pub_odompath, _pub_twist, _pub_mpctraj;
            ros::Timer _timer1;
            tf2_ros::Buffer *tf_;  ///
            ros::Time tracking_stime;
            ros::Time tracking_etime;
            ros::Time tracking_time;
            int tracking_time_sec;
            int tracking_time_nsec;
            

            std::ofstream file;
            unsigned int idx = 0;

            //time flag
            bool start_timef = false;
            bool end_timef = false;

            geometry_msgs::Point _goal_pos;
            nav_msgs::Odometry _odom;
            nav_msgs::Path _odom_path, _mpc_traj; 
            //ackermann_msgs::AckermannDriveStamped _ackermann_msg;
            geometry_msgs::Twist _twist_msg;

            string _globalPath_topic, _goal_topic;
            string _map_frame, _odom_frame, _car_frame;

            MPC _mpc;
            map<string, double> _mpc_params;
            double _mpc_steps, _ref_cte, _ref_etheta, _ref_vel, _w_cte, _w_etheta, _w_vel, 
                _w_angvel, _w_accel, _w_angvel_d, _w_accel_d, _max_angvel, _max_throttle, _bound_value;

            //double _Lf; 
            double _dt, _w, _throttle, _speed, _max_speed;
            double _pathLength, _goalRadius, _waypointsDist;
            int _controller_freq, _downSampling, _thread_numbers;
            bool _goal_received, _goal_reached, _path_computed, _pub_twist_flag, _debug_info, _delay_mode;
            double polyeval(Eigen::VectorXd coeffs, double x);
            Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

            void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
            void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
            void desiredPathCB(const nav_msgs::Path::ConstPtr& pathMsg);
            void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
            void amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg);
            void controlLoopCB(const ros::TimerEvent&);
    };
};
#endif /* MPC_LOCAL_PLANNER_NODE_ROS_H */
