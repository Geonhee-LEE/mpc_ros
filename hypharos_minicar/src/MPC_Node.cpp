/*
# Copyright 2018 HyphaROS Workshop.
# Developer: HaoChih, LIN (hypha.ros@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
*/

#include <iostream>
#include <map>
#include <math.h>

#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>

#include "MPC.h"
#include <Eigen/Core>
#include <Eigen/QR>

using namespace std;
using namespace Eigen;

/********************/
/* CLASS DEFINITION */
/********************/
class MPCNode
{
    public:
        MPCNode();
        int get_thread_numbers();
        
    private:
        ros::NodeHandle _nh;
        ros::Subscriber _sub_odom, _sub_path, _sub_goal, _sub_amcl;
        ros::Publisher _pub_odompath, _pub_twist, _pub_ackermann, _pub_mpctraj;
        ros::Timer _timer1;
        tf::TransformListener _tf_listener;

        geometry_msgs::Point _goal_pos;
        nav_msgs::Odometry _odom;
        nav_msgs::Path _odom_path, _mpc_traj; 
        ackermann_msgs::AckermannDriveStamped _ackermann_msg;
        geometry_msgs::Twist _twist_msg;

        string _globalPath_topic, _goal_topic;
        string _map_frame, _odom_frame, _car_frame;

        MPC _mpc;
        map<string, double> _mpc_params;
        double _mpc_steps, _ref_cte, _ref_epsi, _ref_vel, _w_cte, _w_epsi, _w_vel, 
               _w_delta, _w_accel, _w_delta_d, _w_accel_d, _max_steering, _max_throttle, _bound_value;

        double _Lf, _dt, _steering, _throttle, _speed, _max_speed;
        double _pathLength, _goalRadius, _waypointsDist;
        int _controller_freq, _downSampling, _thread_numbers;
        bool _goal_received, _goal_reached, _path_computed, _pub_twist_flag, _debug_info, _delay_mode;

        double polyeval(Eigen::VectorXd coeffs, double x);
        Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

        void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
        void amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg);
        void controlLoopCB(const ros::TimerEvent&);

}; // end of class


MPCNode::MPCNode()
{
    //Private parameters handler
    ros::NodeHandle pn("~");

    //Parameters for control loop
    pn.param("thread_numbers", _thread_numbers, 2); // number of threads for this ROS node
    pn.param("pub_twist_cmd", _pub_twist_flag, true);
    pn.param("debug_info", _debug_info, false);
    pn.param("delay_mode", _delay_mode, true);
    pn.param("max_speed", _max_speed, 2.0); // unit: m/s
    pn.param("waypoints_dist", _waypointsDist, -1.0); // unit: m
    pn.param("path_length", _pathLength, 8.0); // unit: m
    pn.param("goal_radius", _goalRadius, 0.5); // unit: m
    pn.param("controller_freq", _controller_freq, 10);
    pn.param("vehicle_Lf", _Lf, 0.25); // distance between the front of the vehicle and its center of gravity
    _dt = double(1.0/_controller_freq); // time step duration dt in s 

    //Parameter for MPC solver
    pn.param("mpc_steps", _mpc_steps, 20.0);
    pn.param("mpc_ref_cte", _ref_cte, 0.0);
    pn.param("mpc_ref_epsi", _ref_epsi, 0.0);
    pn.param("mpc_ref_vel", _ref_vel, 1.5);
    pn.param("mpc_w_cte", _w_cte, 100.0);
    pn.param("mpc_w_epsi", _w_epsi, 100.0);
    pn.param("mpc_w_vel", _w_vel, 100.0);
    pn.param("mpc_w_delta", _w_delta, 100.0);
    pn.param("mpc_w_accel", _w_accel, 50.0);
    pn.param("mpc_w_delta_d", _w_delta_d, 0.0);
    pn.param("mpc_w_accel_d", _w_accel_d, 0.0);
    pn.param("mpc_max_steering", _max_steering, 0.523); // Maximal steering radian (~30 deg)
    pn.param("mpc_max_throttle", _max_throttle, 1.0); // Maximal throttle accel
    pn.param("mpc_bound_value", _bound_value, 1.0e3); // Bound value for other variables

    //Parameter for topics & Frame name
    pn.param<std::string>("global_path_topic", _globalPath_topic, "/move_base/TrajectoryPlannerROS/global_plan" );
    pn.param<std::string>("goal_topic", _goal_topic, "/move_base_simple/goal" );
    pn.param<std::string>("map_frame", _map_frame, "map" );
    pn.param<std::string>("odom_frame", _odom_frame, "odom");
    pn.param<std::string>("car_frame", _car_frame, "base_link" );

    //Display the parameters
    cout << "\n===== Parameters =====" << endl;
    cout << "pub_twist_cmd: "  << _pub_twist_flag << endl;
    cout << "debug_info: "  << _debug_info << endl;
    cout << "delay_mode: "  << _delay_mode << endl;
    cout << "vehicle_Lf: "  << _Lf << endl;
    cout << "frequency: "   << _dt << endl;
    cout << "mpc_steps: "   << _mpc_steps << endl;
    cout << "mpc_ref_vel: " << _ref_vel << endl;
    cout << "mpc_w_cte: "   << _w_cte << endl;
    cout << "mpc_w_epsi: "  << _w_epsi << endl;
    cout << "mpc_max_steering: "  << _max_steering << endl;

    //Publishers and Subscribers
    _sub_odom   = _nh.subscribe("/odom", 1, &MPCNode::odomCB, this);
    _sub_path   = _nh.subscribe( _globalPath_topic, 1, &MPCNode::pathCB, this);
    _sub_goal   = _nh.subscribe( _goal_topic, 1, &MPCNode::goalCB, this);
    _sub_amcl   = _nh.subscribe("/amcl_pose", 5, &MPCNode::amclCB, this);
    _pub_odompath  = _nh.advertise<nav_msgs::Path>("/mpc_reference", 1); // reference path for MPC 
    _pub_mpctraj   = _nh.advertise<nav_msgs::Path>("/mpc_trajectory", 1);// MPC trajectory output
    _pub_ackermann = _nh.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 1);
    if(_pub_twist_flag)
        _pub_twist = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1); //for stage (Ackermann msg non-supported)
    
    //Timer
    _timer1 = _nh.createTimer(ros::Duration((1.0)/_controller_freq), &MPCNode::controlLoopCB, this); // 10Hz

    //Init variables
    _goal_received = false;
    _goal_reached  = false;
    _path_computed = false;
    _throttle = 0.0; 
    _steering = 0.0;
    _speed = 0.0;

    _ackermann_msg = ackermann_msgs::AckermannDriveStamped();
    _twist_msg = geometry_msgs::Twist();
    _mpc_traj = nav_msgs::Path();

    //Init parameters for MPC object
    _mpc_params["DT"] = _dt;
    _mpc_params["LF"] = _Lf;
    _mpc_params["STEPS"]    = _mpc_steps;
    _mpc_params["REF_CTE"]  = _ref_cte;
    _mpc_params["REF_EPSI"] = _ref_epsi;
    _mpc_params["REF_V"]    = _ref_vel;
    _mpc_params["W_CTE"]    = _w_cte;
    _mpc_params["W_EPSI"]   = _w_epsi;
    _mpc_params["W_V"]      = _w_vel;
    _mpc_params["W_DELTA"]  = _w_delta;
    _mpc_params["W_A"]      = _w_accel;
    _mpc_params["W_DDELTA"] = _w_delta_d;
    _mpc_params["W_DA"]     = _w_accel_d;
    _mpc_params["MAXSTR"]   = _max_steering;
    _mpc_params["MAXTHR"]   = _max_throttle;
    _mpc_params["BOUND"]    = _bound_value;
    _mpc.LoadParams(_mpc_params);

}


// Public: return _thread_numbers
int MPCNode::get_thread_numbers()
{
    return _thread_numbers;
}


// Evaluate a polynomial.
double MPCNode::polyeval(Eigen::VectorXd coeffs, double x) 
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) 
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}


// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd MPCNode::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) 
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
        A(i, 0) = 1.0;

    for (int j = 0; j < xvals.size(); j++) 
    {
        for (int i = 0; i < order; i++) 
            A(j, i + 1) = A(j, i) * xvals(j);
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

// CallBack: Update odometry
void MPCNode::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    _odom = *odomMsg;
}

// CallBack: Update path waypoints (conversion to odom frame)
void MPCNode::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{
    if(_goal_received && !_goal_reached)
    {    
        nav_msgs::Path odom_path = nav_msgs::Path();
        try
        {
            //find waypoints distance
            if(_waypointsDist <=0.0)
            {        
                double dx = pathMsg->poses[1].pose.position.x - pathMsg->poses[0].pose.position.x;
                double dy = pathMsg->poses[1].pose.position.y - pathMsg->poses[0].pose.position.y;
                _waypointsDist = sqrt(dx*dx + dy*dy);
                _downSampling = int(_pathLength/10.0/_waypointsDist);
            }
            
            double total_length = 0.0;
            int sampling = _downSampling;
            // Cut and downsampling the path
            for(int i =0; i< pathMsg->poses.size(); i++)
            {
                if(total_length > _pathLength)
                    break;

                if(sampling == _downSampling)
                {   
                    geometry_msgs::PoseStamped tempPose;
                    _tf_listener.transformPose(_odom_frame, ros::Time(0) , pathMsg->poses[i], _map_frame, tempPose);                     
                    odom_path.poses.push_back(tempPose);  
                    sampling = 0;
                }
                total_length = total_length + _waypointsDist; 
                sampling = sampling + 1;  
            }
           
            if(odom_path.poses.size() >= 6 )
            {
                _odom_path = odom_path; // Path waypoints in odom frame
                _path_computed = true;
                // publish odom path
                odom_path.header.frame_id = _odom_frame;
                odom_path.header.stamp = ros::Time::now();
                _pub_odompath.publish(odom_path);
            }
            //DEBUG            
            //cout << endl << "N: " << odom_path.poses.size() << endl <<  "Car path[0]: " << odom_path.poses[0] << ", path[N]: " << _odom_path.poses[_odom_path.poses.size()-1] << endl;


        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }
}

// CallBack: Update goal status
void MPCNode::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    _goal_pos = goalMsg->pose.position;
    _goal_received = true;
    _goal_reached = false;
    ROS_INFO("Goal Received !");
}


// Callback: Check if the car is inside the goal area or not 
void MPCNode::amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg)
{

    if(_goal_received)
    {
        double car2goal_x = _goal_pos.x - amclMsg->pose.pose.position.x;
        double car2goal_y = _goal_pos.y - amclMsg->pose.pose.position.y;
        double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);
        if(dist2goal < _goalRadius)
        {
            _goal_reached = true;
            _goal_received = false;
            _path_computed = false;
            ROS_INFO("Goal Reached !");
        }
    }
}


// Timer: Control Loop (closed loop nonlinear MPC)
void MPCNode::controlLoopCB(const ros::TimerEvent&)
{      
    if(_goal_received && !_goal_reached && _path_computed ) //received goal & goal not reached    
    {    
        nav_msgs::Odometry odom = _odom; 
        nav_msgs::Path odom_path = _odom_path;   

        // Update system states: X=[x, y, psi, v]
        const double px = odom.pose.pose.position.x; //pose: odom frame
        const double py = odom.pose.pose.position.y;
        tf::Pose pose;
        tf::poseMsgToTF(odom.pose.pose, pose);
        const double psi = tf::getYaw(pose.getRotation());
        const double v = odom.twist.twist.linear.x; //twist: body fixed frame
        // Update system inputs: U=[steering, throttle]
        const double steering = _steering;  // radian
        const double throttle = _throttle; // accel: >0; brake: <0
        const double dt = _dt;
        const double Lf = _Lf;

        // Waypoints related parameters
        const int N = odom_path.poses.size(); // Number of waypoints
        const double cospsi = cos(psi);
        const double sinpsi = sin(psi);

        // Convert to the vehicle coordinate system
        VectorXd x_veh(N);
        VectorXd y_veh(N);
        for(int i = 0; i < N; i++) 
        {
            const double dx = odom_path.poses[i].pose.position.x - px;
            const double dy = odom_path.poses[i].pose.position.y - py;
            x_veh[i] = dx * cospsi + dy * sinpsi;
            y_veh[i] = dy * cospsi - dx * sinpsi;
        }
        
        // Fit waypoints
        auto coeffs = polyfit(x_veh, y_veh, 3); 

        const double cte  = polyeval(coeffs, 0.0);
        const double epsi = atan(coeffs[1]);
        VectorXd state(6);
        if(_delay_mode)
        {
            // Kinematic model is used to predict vehicle state at the actual
            // moment of control (current time + delay dt)
            const double px_act = v * dt;
            const double py_act = 0;
            const double psi_act = v * steering * dt / Lf;
            const double v_act = v + throttle * dt;
            const double cte_act = cte + v * sin(epsi) * dt;
            const double epsi_act = -epsi + psi_act;             
            state << px_act, py_act, psi_act, v_act, cte_act, epsi_act;
        }
        else
        {
            state << 0, 0, 0, v, cte, epsi;
        }
        
        // Solve MPC Problem
        vector<double> mpc_results = _mpc.Solve(state, coeffs);
              
        // MPC result (all described in car frame)        
        _steering = mpc_results[0]; // radian
        _throttle = mpc_results[1]; // acceleration
        _speed = v + _throttle*dt;  // speed
        if (_speed >= _max_speed)
            _speed = _max_speed;
        if(_speed <= 0.0)
            _speed = 0.0;

        if(_debug_info)
        {
            cout << "\n\nDEBUG" << endl;
            cout << "psi: " << psi << endl;
            cout << "V: " << v << endl;
            //cout << "odom_path: \n" << odom_path << endl;
            //cout << "x_points: \n" << x_veh << endl;
            //cout << "y_points: \n" << y_veh << endl;
            cout << "coeffs: \n" << coeffs << endl;
            cout << "_steering: \n" << _steering << endl;
            cout << "_throttle: \n" << _throttle << endl;
            cout << "_speed: \n" << _speed << endl;
        }

        // Display the MPC predicted trajectory
        _mpc_traj = nav_msgs::Path();
        _mpc_traj.header.frame_id = _car_frame; // points in car coordinate        
        _mpc_traj.header.stamp = ros::Time::now();
        for(int i=0; i<_mpc.mpc_x.size(); i++)
        {
            geometry_msgs::PoseStamped tempPose;
            tempPose.header = _mpc_traj.header;
            tempPose.pose.position.x = _mpc.mpc_x[i];
            tempPose.pose.position.y = _mpc.mpc_y[i];
            tempPose.pose.orientation.w = 1.0;
            _mpc_traj.poses.push_back(tempPose); 
        }     
        // publish the mpc trajectory
        _pub_mpctraj.publish(_mpc_traj);

    }
    else
    {
        _steering = 0.0;
        _throttle = 0.0;
        _speed = 0.0;
        if(_goal_reached && _goal_received)
            cout << "Goal Reached !" << endl;
    }

    // publish cmd 
    _ackermann_msg.header.frame_id = _car_frame;
    _ackermann_msg.header.stamp = ros::Time::now();
    _ackermann_msg.drive.steering_angle = _steering;
    _ackermann_msg.drive.speed = _speed;
    _ackermann_msg.drive.acceleration = _throttle;
    _pub_ackermann.publish(_ackermann_msg);        

    if(_pub_twist_flag)
    {
        _twist_msg.linear.x  = _speed; 
        _twist_msg.angular.z = _steering;
        _pub_twist.publish(_twist_msg);
    }
    
}


/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "MPC_Node");
    MPCNode mpc_node;

    ROS_INFO("Waiting for global path msgs ~");
    ros::AsyncSpinner spinner(mpc_node.get_thread_numbers()); // Use multi threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}

