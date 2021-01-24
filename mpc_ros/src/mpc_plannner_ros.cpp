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

#include "mpc_plannner_ros.h"
#include <pluginlib/class_list_macros.h>

using namespace std;
using namespace Eigen;

PLUGINLIB_EXPORT_CLASS(mpc_ros::MPCPlannerROS, nav_core::BaseLocalPlanner)

namespace mpc_ros{

    MPCPlannerROS::MPCPlannerROS() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}
	MPCPlannerROS::MPCPlannerROS(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), tf_(NULL), initialized_(false)
    {
        // initialize planner
        initialize(name, tf, costmap_ros);
    }
	MPCPlannerROS::~MPCPlannerROS() {}

	void MPCPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){

        ros::NodeHandle private_nh("~/" + name);
        g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
        l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);

		tf_ = tf;
		costmap_ros_ = costmap_ros;
        //initialize the copy of the costmap the controller will use
        costmap_ = costmap_ros_->getCostmap();
        global_frame_ = costmap_ros_->getGlobalFrameID();
        robot_base_frame_ = costmap_ros_->getBaseFrameID();
        footprint_spec_ = costmap_ros_->getRobotFootprint();
        
        planner_util_.initialize(tf, costmap_, costmap_ros_->getGlobalFrameID());
        
        //Private parameters handler
        ros::NodeHandle pn("~");

        //Parameters for control loop
        pn.param("thread_numbers", _thread_numbers, 2); // number of threads for this ROS node
        pn.param("pub_twist_cmd", _pub_twist_flag, true);
        pn.param("debug_info", _debug_info, true);
        pn.param("delay_mode", _delay_mode, true);
        pn.param("max_speed", _max_speed, 0.50); // unit: m/s
        pn.param("waypoints_dist", _waypointsDist, -1.0); // unit: m
        pn.param("path_length", _pathLength, 8.0); // unit: m
        pn.param("goal_radius", _goalRadius, 0.5); // unit: m
        pn.param("controller_freq", _controller_freq, 10);
        //pn.param("vehicle_Lf", _Lf, 0.290); // distance between the front of the vehicle and its center of gravity
        _dt = double(1.0/_controller_freq); // time step duration dt in s 

        //Parameter for MPC solver
        pn.param("mpc_steps", _mpc_steps, 20.0);
        pn.param("mpc_ref_cte", _ref_cte, 0.0);
        pn.param("mpc_ref_vel", _ref_vel, 1.0);
        pn.param("mpc_ref_etheta", _ref_etheta, 0.0);
        pn.param("mpc_w_cte", _w_cte, 5000.0);
        pn.param("mpc_w_etheta", _w_etheta, 5000.0);
        pn.param("mpc_w_vel", _w_vel, 1.0);
        pn.param("mpc_w_angvel", _w_angvel, 100.0);
        pn.param("mpc_w_angvel_d", _w_angvel_d, 10.0);
        pn.param("mpc_w_accel", _w_accel, 50.0);
        pn.param("mpc_w_accel_d", _w_accel_d, 10.0);
        pn.param("mpc_max_angvel", _max_angvel, 3.0); // Maximal angvel radian (~30 deg)
        pn.param("mpc_max_throttle", _max_throttle, 1.0); // Maximal throttle accel
        pn.param("mpc_bound_value", _bound_value, 1.0e3); // Bound value for other variables

        //Parameter for topics & Frame name
        pn.param<std::string>("global_path_topic", _globalPath_topic, "/move_base/TrajectoryPlannerROS/global_plan" );
        pn.param<std::string>("goal_topic", _goal_topic, "/move_base_simple/goal" );
        pn.param<std::string>("map_frame", _map_frame, "map" ); //*****for mpc, "odom"
        pn.param<std::string>("odom_frame", _odom_frame, "odom");
        pn.param<std::string>("car_frame", _car_frame, "base_footprint" );

        //Display the parameters
        cout << "\n===== Parameters =====" << endl;
        cout << "pub_twist_cmd: "  << _pub_twist_flag << endl;
        cout << "debug_info: "  << _debug_info << endl;
        cout << "delay_mode: "  << _delay_mode << endl;
        //cout << "vehicle_Lf: "  << _Lf << endl;
        cout << "frequency: "   << _dt << endl;
        cout << "mpc_steps: "   << _mpc_steps << endl;
        cout << "mpc_ref_vel: " << _ref_vel << endl;
        cout << "mpc_w_cte: "   << _w_cte << endl;
        cout << "mpc_w_etheta: "  << _w_etheta << endl;
        cout << "mpc_max_angvel: "  << _max_angvel << endl;

        //Publishers and Subscribers
        /*
        _sub_odom   = _nh.subscribe("/odom", 1, &MPCPlannerROS::odomCB, this);
        _sub_path   = _nh.subscribe( _globalPath_topic, 1, &MPCPlannerROS::pathCB, this);
        _sub_goal   = _nh.subscribe( _goal_topic, 1, &MPCPlannerROS::goalCB, this);
        _sub_amcl   = _nh.subscribe("/amcl_pose", 5, &MPCPlannerROS::amclCB, this);
        _pub_globalpath  = _nh.advertise<nav_msgs::Path>("/global_path", 1); // Global path generated from another source
        _pub_odompath  = _nh.advertise<nav_msgs::Path>("/mpc_reference", 1); // reference path for MPC ///mpc_reference 
        _pub_mpctraj   = _nh.advertise<nav_msgs::Path>("/mpc_trajectory", 1);// MPC trajectory output
        //Timer
        _timer1 = _nh.createTimer(ros::Duration((1.0)/_controller_freq), &MPCPlannerROS::controlLoopCB, this); // 10Hz //*****mpc
        */
        if(_pub_twist_flag)
            _pub_twist = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1); //for stage (Ackermann msg non-supported)
        

        //Init variables
        _goal_received = false;
        _goal_reached  = false;
        _path_computed = false;
        _throttle = 0.0; 
        _w = 0.0;
        _speed = 0.0;

        //_ackermann_msg = ackermann_msgs::AckermannDriveStamped();
        _twist_msg = geometry_msgs::Twist();
        _mpc_traj = nav_msgs::Path();



        idx = 0;
        file.open("/home/nscl1016/catkin_ws/src/mpc_ros/mpc.csv");
        file << "idx"<< "," << "cte" << "," <<  "etheta" << "," << "cmd_vel.linear.x" << "," << "cmd_vel.angular.z" << "\n";


        //Init parameters for MPC object
        _mpc_params["DT"] = _dt;
        //_mpc_params["LF"] = _Lf;
        _mpc_params["STEPS"]    = _mpc_steps;
        _mpc_params["REF_CTE"]  = _ref_cte;
        _mpc_params["REF_ETHETA"] = _ref_etheta;
        _mpc_params["REF_V"]    = _ref_vel;
        _mpc_params["W_CTE"]    = _w_cte;
        _mpc_params["W_EPSI"]   = _w_etheta;
        _mpc_params["W_V"]      = _w_vel;
        _mpc_params["W_ANGVEL"]  = _w_angvel;
        _mpc_params["W_A"]      = _w_accel;
        _mpc_params["W_DANGVEL"] = _w_angvel_d;
        _mpc_params["W_DA"]     = _w_accel_d;
        _mpc_params["ANGVEL"]   = _max_angvel;
        _mpc_params["MAXTHR"]   = _max_throttle;
        _mpc_params["BOUND"]    = _bound_value;
        _mpc.LoadParams(_mpc_params);

        initialized_ = true;
    }

    void MPCPlannerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
        base_local_planner::publishPlan(path, l_plan_pub_);
    }

    void MPCPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
        base_local_planner::publishPlan(path, g_plan_pub_);
    }
  
	bool MPCPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
        if( ! isInitialized()) {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        latchedStopRotateController_.resetLatching();
        reached_goal_ = false;
        planner_util_.setPlan(orig_global_plan);
        
    }

    void MPCPlannerROS::updatePlanAndLocalCosts(
        const geometry_msgs::PoseStamped& global_pose,
        const std::vector<geometry_msgs::PoseStamped>& new_plan,
        const std::vector<geometry_msgs::Point>& footprint_spec) {
        
        global_plan_.resize(new_plan.size());
        for (unsigned int i = 0; i < new_plan.size(); ++i) {
            global_plan_[i] = new_plan[i];
        }
    }

	bool MPCPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
        // dispatches to either dwa sampling control or stop and rotate control, depending on whether we have been close enough to goal
        if ( ! costmap_ros_->getRobotPose(current_pose_)) {
            ROS_ERROR("Could not get robot pose");
            return false;
        }
        std::vector<geometry_msgs::PoseStamped> transformed_plan;
        if ( ! planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
            ROS_ERROR("Could not get local plan");
            return false;
        }
        //if the global plan passed in is empty... we won't do anything
        if(transformed_plan.empty()) {
            ROS_WARN_NAMED("mpc_planner", "Received an empty transformed plan.");
            return false;
        }
        ROS_DEBUG_NAMED("mpc_planner", "Received a transformed plan with %zu points.", transformed_plan.size());
        updatePlanAndLocalCosts(current_pose_, transformed_plan, costmap_ros_->getRobotFootprint());

        /*if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_)) {
            //publish an empty plan because we've reached our goal position
            std::vector<geometry_msgs::PoseStamped> local_plan;
            std::vector<geometry_msgs::PoseStamped> transformed_plan;
            publishGlobalPlan(transformed_plan);
            publishLocalPlan(local_plan);
            base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
            return latchedStopRotateController_.computeVelocityCommandsStopRotate(
                cmd_vel,
                limits.getAccLimits(),
                dp_->getSimPeriod(),
                &planner_util_,
                odom_helper_,
                current_pose_,
                boost::bind(&DWAPlanner::checkTrajectory, dp_, _1, _2, _3));
        } else */{
            bool isOk = mpcComputeVelocityCommands(current_pose_, cmd_vel);
            if (isOk) {
                publishGlobalPlan(transformed_plan);
            } else {
                ROS_WARN_NAMED("mpc_ros", "MPC Planner failed to produce path.");
                std::vector<geometry_msgs::PoseStamped> empty_plan;
                publishGlobalPlan(empty_plan);
            }
            return isOk;
        }
    }

    // Timer: Control Loop (closed loop nonlinear MPC)
    bool MPCPlannerROS::mpcComputeVelocityCommands(geometry_msgs::PoseStamped global_pose, geometry_msgs::Twist& cmd_vel)
    {         
        // dynamic window sampling approach to get useful velocity commands
        if(! isInitialized()){
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        geometry_msgs::PoseStamped robot_vel;
        odom_helper_.getRobotVel(robot_vel);

        //compute what trajectory to drive along
        geometry_msgs::PoseStamped drive_cmds;
        drive_cmds.header.frame_id = costmap_ros_->getBaseFrameID();


        // call with updated footprint
        base_local_planner::Trajectory path = findBestPath(global_pose, robot_vel, drive_cmds);
        //base_local_planner::Trajectory path = dp_->findBestPath(global_pose, robot_vel, drive_cmds);
        //ROS_ERROR("Best: %.2f, %.2f, %.2f, %.2f", path.xv_, path.yv_, path.thetav_, path.cost_);

        //pass along drive commands
        cmd_vel.linear.x = drive_cmds.pose.position.x;
        cmd_vel.linear.y = drive_cmds.pose.position.y;
        cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation);

        //if we cannot move... tell someone
        std::vector<geometry_msgs::PoseStamped> local_plan;
        if(path.cost_ < 0) {
            ROS_DEBUG_NAMED("mpc_ros",
                "The dwa local planner failed to find a valid plan, cost functions discarded all candidates. This can mean there is an obstacle too close to the robot.");
            local_plan.clear();
            publishLocalPlan(local_plan);
            return false;
        }

        ROS_DEBUG_NAMED("mpc_ros", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.", 
                        cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

        // Fill out the local plan
        for(unsigned int i = 0; i < path.getPointsSize(); ++i) {
            double p_x, p_y, p_th;
            path.getPoint(i, p_x, p_y, p_th);

            geometry_msgs::PoseStamped p;
            p.header.frame_id = costmap_ros_->getGlobalFrameID();
            p.header.stamp = ros::Time::now();
            p.pose.position.x = p_x;
            p.pose.position.y = p_y;
            p.pose.position.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, p_th);
            tf2::convert(q, p.pose.orientation);
            local_plan.push_back(p);
        }

        //publish information to the visualizer

        publishLocalPlan(local_plan);
        return true;
    }

    base_local_planner::Trajectory MPCPlannerROS::findBestPath(
      const geometry_msgs::PoseStamped& global_pose,
      const geometry_msgs::PoseStamped& global_vel,
      geometry_msgs::PoseStamped& drive_velocities){

        base_local_planner::Trajectory result_traj_;

        Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation));
        Eigen::Vector3f vel(global_vel.pose.position.x, global_vel.pose.position.y, tf2::getYaw(global_vel.pose.orientation));
        geometry_msgs::PoseStamped goal_pose = global_plan_.back();
        Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf2::getYaw(goal_pose.pose.orientation));
        base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
        result_traj_.cost_ = -7;

        // http://docs.ros.org/en/jade/api/base_local_planner/html/classbase__local__planner_1_1SimpleTrajectoryGenerator.html#a0810ac35a39d3d7ccc1c19a862e97fbf
        // prepare cost functions and generators for this run
        Eigen::Vector3f vsamples_;
        int vx_samp, vy_samp, vth_samp;
        if (vx_samp <= 0) {
        ROS_WARN("You've specified that you don't want any samples in the x dimension. We'll at least assume that you want to sample one value... so we're going to set vx_samples to 1 instead");
        vx_samp = 1;
        }
    
        if (vy_samp <= 0) {
        ROS_WARN("You've specified that you don't want any samples in the y dimension. We'll at least assume that you want to sample one value... so we're going to set vy_samples to 1 instead");
        vy_samp = 1;
        }
    
        if (vth_samp <= 0) {
        ROS_WARN("You've specified that you don't want any samples in the th dimension. We'll at least assume that you want to sample one value... so we're going to set vth_samples to 1 instead");
        vth_samp = 1;
        }
        vsamples_[0] = vx_samp;
        vsamples_[1] = vy_samp;
        vsamples_[2] = vth_samp;
        generator_.initialise(pos,
            vel,
            goal,
            &limits,
            vsamples_);

        if(_goal_received && ! _goal_reached && _path_computed ) //received goal & goal not reached    
        {
            if( ! start_timef)
            {
                tracking_stime == ros::Time::now();
                start_timef = true;
            }
            nav_msgs::Path odom_path = _odom_path;   
            geometry_msgs::Point goal_pos = _goal_pos;

            // Update system states: X=[x, y, theta, v]
            const double px = global_pose.pose.position.x; 
            const double py = global_pose.pose.position.y;
            tf::Pose pose;
            tf::poseMsgToTF(global_pose.pose, pose);
            double theta = tf::getYaw(pose.getRotation());
            const double v = global_vel.pose.position.x; //twist: body fixed frame
            // Update system inputs: U=[w, throttle]
            const double w = _w; // steering -> w
            //const double steering = _steering;  // radian
            const double throttle = _throttle; // accel: >0; brake: <0
            const double dt = _dt;
            //const double Lf = _Lf;

            // Waypoints related parameters
            const int N = odom_path.poses.size(); // Number of waypoints
            const double costheta = cos(theta);
            const double sintheta = sin(theta);

            // Convert to the vehicle coordinate system
            VectorXd x_veh(N);
            VectorXd y_veh(N);
            for(int i = 0; i < N; i++) 
            {
                const double dx = odom_path.poses[i].pose.position.x - px;
                const double dy = odom_path.poses[i].pose.position.y - py;
                x_veh[i] = dx * costheta + dy * sintheta;
                y_veh[i] = dy * costheta - dx * sintheta;
            }

            // Fit waypoints
            auto coeffs = polyfit(x_veh, y_veh, 3); 
            const double cte  = polyeval(coeffs, 0.0);
            cout << "coeffs : " << coeffs[0] << endl;
            cout << "pow : " << pow(0.0 ,0) << endl;
            cout << "cte : " << cte << endl;
            double etheta = atan(coeffs[1]);

            // Global coordinate system about theta
            double gx = 0;
            double gy = 0;
            int N_sample = N * 0.3;
            for(int i = 1; i < N_sample; i++) 
            {
                gx += odom_path.poses[i].pose.position.x - odom_path.poses[i-1].pose.position.x;
                gy += odom_path.poses[i].pose.position.y - odom_path.poses[i-1].pose.position.y;
            }       
            
            double temp_theta = theta;
            double traj_deg = atan2(gy,gx);
            double PI = 3.141592;

            // Degree conversion -pi~pi -> 0~2pi(ccw) since need a continuity        
            if(temp_theta <= -PI + traj_deg) 
                temp_theta = temp_theta + 2 * PI;
            
            // Implementation about theta error more precisly
            if(gx && gy && temp_theta - traj_deg < 1.8 * PI)
                etheta = temp_theta - traj_deg;
            else
                etheta = 0;

            cout << "etheta: "<< etheta << ", atan2(gy,gx): " << atan2(gy,gx) << ", temp_theta:" << traj_deg << endl;

            idx++;
            file << idx<< "," << cte << "," <<  etheta << "," << _twist_msg.linear.x << "," << _twist_msg.angular.z << "\n";
            
            // Difference bewteen current position and goal position
            const double x_err = goal_pos.x - global_pose.pose.position.x;
            const double y_err = goal_pos.y - global_pose.pose.position.y;
            const double goal_err = sqrt(x_err*x_err + y_err*y_err);

            cout << "x_err:"<< x_err << ", y_err:"<< y_err  << endl;

            VectorXd state(6);
            if(_delay_mode)
            {
                // Kinematic model is used to predict vehicle state at the actual moment of control (current time + delay dt)
                const double px_act = v * dt;
                const double py_act = 0;
                const double theta_act = w * dt; //(steering) theta_act = v * steering * dt / Lf;
                const double v_act = v + throttle * dt; //v = v + a * dt
                
                const double cte_act = cte + v * sin(etheta) * dt;
                const double etheta_act = etheta - theta_act;  
                
                state << px_act, py_act, theta_act, v_act, cte_act, etheta_act;
            }
            else
            {
                state << 0, 0, 0, v, cte, etheta;
            }
            
            // Solve MPC Problem
            ros::Time begin = ros::Time::now();
            vector<double> mpc_results = _mpc.Solve(state, coeffs);    
            ros::Time end = ros::Time::now();
            cout << "Duration: " << end.sec << "." << end.nsec << endl << begin.sec<< "."  << begin.nsec << endl;
                
            // MPC result (all described in car frame), output = (acceleration, w)        
            _w = mpc_results[0]; // radian/sec, angular velocity
            _throttle = mpc_results[1]; // acceleration

            _speed = v + _throttle * dt;  // speed
            if (_speed >= _max_speed)
                _speed = _max_speed;
            if(_speed <= 0.0)
                _speed = 0.0;

            if(_debug_info)
            {
                cout << "\n\nDEBUG" << endl;
                cout << "theta: " << theta << endl;
                cout << "V: " << v << endl;
                //cout << "odom_path: \n" << odom_path << endl;
                //cout << "x_points: \n" << x_veh << endl;
                //cout << "y_points: \n" << y_veh << endl;
                cout << "coeffs: \n" << coeffs << endl;
                cout << "_w: \n" << _w << endl;
                cout << "_throttle: \n" << _throttle << endl;
                cout << "_speed: \n" << _speed << endl;
            }

            // Display the MPC predicted trajectory
            _mpc_traj = nav_msgs::Path();
            _mpc_traj.header.frame_id = _car_frame; // points in car coordinate        
            _mpc_traj.header.stamp = ros::Time::now();

            geometry_msgs::PoseStamped tempPose;
            tf2::Quaternion myQuaternion;

            for(int i=0; i<_mpc.mpc_x.size(); i++)
            {
                tempPose.header = _mpc_traj.header;
                tempPose.pose.position.x = _mpc.mpc_x[i];
                tempPose.pose.position.y = _mpc.mpc_y[i];

                myQuaternion.setRPY( 0, 0, _mpc.mpc_theta[i] );  
                tempPose.pose.orientation.x = myQuaternion[0];
                tempPose.pose.orientation.y = myQuaternion[1];
                tempPose.pose.orientation.z = myQuaternion[2];
                tempPose.pose.orientation.w = myQuaternion[3];
                    
                _mpc_traj.poses.push_back(tempPose); 
            }     
            // publish the mpc trajectory
            _pub_mpctraj.publish(_mpc_traj);
        }
        else
        {
            _throttle = 0.0;
            _speed = 0.0;
            _w = 0;
            if(_goal_reached && _goal_received)
            {
                cout << "Goal Reached: control loop !" << endl;
            }
        }
        // publish general cmd_vel 
        if(_pub_twist_flag)
        {
            _twist_msg.linear.x  = _speed; 
            _twist_msg.angular.z = _w;
            _pub_twist.publish(_twist_msg);
        }

        return result_traj_;
    }

	bool MPCPlannerROS::isGoalReached(){
        if( ! isInitialized()) {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        if ( ! costmap_ros_->getRobotPose(current_pose_)) {
            ROS_ERROR("Could not get robot pose");
            return false;
        }

        if(latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_)) {
            ROS_INFO("Goal reached");
            return true;
        } else {
            return false;
        }
    }

    // Public: return _thread_numbers
    int MPCPlannerROS::get_thread_numbers()
    {
        return _thread_numbers;
    }

    // Evaluate a polynomial.
    double MPCPlannerROS::polyeval(Eigen::VectorXd coeffs, double x) 
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
    Eigen::VectorXd MPCPlannerROS::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) 
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
    void MPCPlannerROS::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        _odom = *odomMsg;
    }

    // CallBack: Update path waypoints (conversion to odom frame)
    void MPCPlannerROS::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
    {
        if(_goal_received && ! _goal_reached)
        {    
            nav_msgs::Path odom_path = nav_msgs::Path();
            try
            {
                double total_length = 0.0;
                int sampling = _downSampling;

                //find waypoints distance
                if(_waypointsDist <=0.0)
                {        
                    double dx = pathMsg->poses[1].pose.position.x - pathMsg->poses[0].pose.position.x;
                    double dy = pathMsg->poses[1].pose.position.y - pathMsg->poses[0].pose.position.y;
                    _waypointsDist = sqrt(dx*dx + dy*dy);
                    _downSampling = int(_pathLength/10.0/_waypointsDist);
                }            

                // Cut and downsampling the path
                for(int i =0; i< pathMsg->poses.size(); i++)
                {
                    if(total_length > _pathLength)
                        break;

                    if(sampling == _downSampling)
                    {   
                        geometry_msgs::PoseStamped tempPose;
                        tf2_ros::TransformListener tfListener(*tf_);
                        geometry_msgs::TransformStamped odom_transform; // My frames are named "base_link" and "leap_motion"
                        odom_transform = tf_->lookupTransform(_odom_frame, _map_frame, ros::Time(0), ros::Duration(1.0) );
                        tf2::doTransform(pathMsg->poses[i], tempPose, odom_transform); // robot_pose is the PoseSta

                        //tfListener.transformPose(_odom_frame, ros::Time(0) , pathMsg->poses[i], _map_frame, tempPose);                     
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
                else
                {
                    cout << "Failed to path generation" << endl;
                    _waypointsDist = -1;
                }
                //DEBUG            
                //cout << endl << "N: " << odom_path.poses.size() << endl 
                //<<  "Car path[0]: " << odom_path.poses[0];
                // << ", path[N]: " << _odom_path.poses[_odom_path.poses.size()-1] << endl;
            }
            catch(tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
        }
    }

    // CallBack: Update goal status
    void MPCPlannerROS::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
    {
        _goal_pos = goalMsg->pose.position;
        _goal_received = true;
        _goal_reached = false;
        ROS_INFO("Goal Received :goalCB!");
    }

    // Callback: Check if the car is inside the goal area or not 
    void MPCPlannerROS::amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg)
    {
        if(_goal_received)
        {
            double car2goal_x = _goal_pos.x - amclMsg->pose.pose.position.x;
            double car2goal_y = _goal_pos.y - amclMsg->pose.pose.position.y;
            double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);
            if(dist2goal < _goalRadius)
            {
                if(start_timef)
                {
                    tracking_etime = ros::Time::now();
                    tracking_time_sec = tracking_etime.sec - tracking_stime.sec; 
                    tracking_time_nsec = tracking_etime.nsec - tracking_stime.nsec; 
                    
                                    
                    file << "tracking time"<< "," << tracking_time_sec << "," <<  tracking_time_nsec << "\n";

                    file.close();

                    
                    start_timef = false;
                    
                }
                _goal_received = false;
                _goal_reached = true;
                _path_computed = false;
                ROS_INFO("Goal Reached !");
                cout << "tracking time: " << tracking_time_sec << "." << tracking_time_nsec << endl;

            }
        }
    }

    // Timer: Control Loop (closed loop nonlinear MPC)
    void MPCPlannerROS::controlLoopCB(const ros::TimerEvent&)
    {          
        if(_goal_received && ! _goal_reached && _path_computed ) //received goal & goal not reached    
        {
            if( ! start_timef)
            {
                tracking_stime == ros::Time::now();
                start_timef = true;
            }
            nav_msgs::Odometry odom = _odom; 
            nav_msgs::Path odom_path = _odom_path;   
            geometry_msgs::Point goal_pos = _goal_pos;

            // Update system states: X=[x, y, theta, v]
            const double px = odom.pose.pose.position.x; //pose: odom frame
            const double py = odom.pose.pose.position.y;
            tf::Pose pose;
            tf::poseMsgToTF(odom.pose.pose, pose);
            double theta = tf::getYaw(pose.getRotation());
            const double v = odom.twist.twist.linear.x; //twist: body fixed frame
            // Update system inputs: U=[w, throttle]
            const double w = _w; // steering -> w
            //const double steering = _steering;  // radian
            const double throttle = _throttle; // accel: >0; brake: <0
            const double dt = _dt;
            //const double Lf = _Lf;

            // Waypoints related parameters
            const int N = odom_path.poses.size(); // Number of waypoints
            const double costheta = cos(theta);
            const double sintheta = sin(theta);

            // Convert to the vehicle coordinate system
            VectorXd x_veh(N);
            VectorXd y_veh(N);
            for(int i = 0; i < N; i++) 
            {
                const double dx = odom_path.poses[i].pose.position.x - px;
                const double dy = odom_path.poses[i].pose.position.y - py;
                x_veh[i] = dx * costheta + dy * sintheta;
                y_veh[i] = dy * costheta - dx * sintheta;
            }

            // Fit waypoints
            auto coeffs = polyfit(x_veh, y_veh, 3); 
            const double cte  = polyeval(coeffs, 0.0);
            cout << "coeffs : " << coeffs[0] << endl;
            cout << "pow : " << pow(0.0 ,0) << endl;
            cout << "cte : " << cte << endl;
            double etheta = atan(coeffs[1]);

            // Global coordinate system about theta
            double gx = 0;
            double gy = 0;
            int N_sample = N * 0.3;
            for(int i = 1; i < N_sample; i++) 
            {
                gx += odom_path.poses[i].pose.position.x - odom_path.poses[i-1].pose.position.x;
                gy += odom_path.poses[i].pose.position.y - odom_path.poses[i-1].pose.position.y;
            }       
            
            double temp_theta = theta;
            double traj_deg = atan2(gy,gx);
            double PI = 3.141592;

            // Degree conversion -pi~pi -> 0~2pi(ccw) since need a continuity        
            if(temp_theta <= -PI + traj_deg) 
                temp_theta = temp_theta + 2 * PI;
            
            // Implementation about theta error more precisly
            if(gx && gy && temp_theta - traj_deg < 1.8 * PI)
                etheta = temp_theta - traj_deg;
            else
                etheta = 0;

            cout << "etheta: "<< etheta << ", atan2(gy,gx): " << atan2(gy,gx) << ", temp_theta:" << traj_deg << endl;


            
            idx++;
            file << idx<< "," << cte << "," <<  etheta << "," << _twist_msg.linear.x << "," << _twist_msg.angular.z << "\n";
            


            // Difference bewteen current position and goal position
            const double x_err = goal_pos.x -  odom.pose.pose.position.x;
            const double y_err = goal_pos.y -  odom.pose.pose.position.y;
            const double goal_err = sqrt(x_err*x_err + y_err*y_err);

            cout << "x_err:"<< x_err << ", y_err:"<< y_err  << endl;

            VectorXd state(6);
            if(_delay_mode)
            {
                // Kinematic model is used to predict vehicle state at the actual moment of control (current time + delay dt)
                const double px_act = v * dt;
                const double py_act = 0;
                const double theta_act = w * dt; //(steering) theta_act = v * steering * dt / Lf;
                const double v_act = v + throttle * dt; //v = v + a * dt
                
                const double cte_act = cte + v * sin(etheta) * dt;
                const double etheta_act = etheta - theta_act;  
                
                state << px_act, py_act, theta_act, v_act, cte_act, etheta_act;
            }
            else
            {
                state << 0, 0, 0, v, cte, etheta;
            }
            
            // Solve MPC Problem
            ros::Time begin = ros::Time::now();
            vector<double> mpc_results = _mpc.Solve(state, coeffs);    
            ros::Time end = ros::Time::now();
            cout << "Duration: " << end.sec << "." << end.nsec << endl << begin.sec<< "."  << begin.nsec << endl;
                
            // MPC result (all described in car frame), output = (acceleration, w)        
            _w = mpc_results[0]; // radian/sec, angular velocity
            _throttle = mpc_results[1]; // acceleration

            _speed = v + _throttle * dt;  // speed
            if (_speed >= _max_speed)
                _speed = _max_speed;
            if(_speed <= 0.0)
                _speed = 0.0;

            if(_debug_info)
            {
                cout << "\n\nDEBUG" << endl;
                cout << "theta: " << theta << endl;
                cout << "V: " << v << endl;
                //cout << "odom_path: \n" << odom_path << endl;
                //cout << "x_points: \n" << x_veh << endl;
                //cout << "y_points: \n" << y_veh << endl;
                cout << "coeffs: \n" << coeffs << endl;
                cout << "_w: \n" << _w << endl;
                cout << "_throttle: \n" << _throttle << endl;
                cout << "_speed: \n" << _speed << endl;
            }

            // Display the MPC predicted trajectory
            _mpc_traj = nav_msgs::Path();
            _mpc_traj.header.frame_id = _car_frame; // points in car coordinate        
            _mpc_traj.header.stamp = ros::Time::now();

            geometry_msgs::PoseStamped tempPose;
            tf2::Quaternion myQuaternion;

            for(int i=0; i<_mpc.mpc_x.size(); i++)
            {
                tempPose.header = _mpc_traj.header;
                tempPose.pose.position.x = _mpc.mpc_x[i];
                tempPose.pose.position.y = _mpc.mpc_y[i];

                myQuaternion.setRPY( 0, 0, _mpc.mpc_theta[i] );  
                tempPose.pose.orientation.x = myQuaternion[0];
                tempPose.pose.orientation.y = myQuaternion[1];
                tempPose.pose.orientation.z = myQuaternion[2];
                tempPose.pose.orientation.w = myQuaternion[3];
                    
                _mpc_traj.poses.push_back(tempPose); 
            }     
            // publish the mpc trajectory
            _pub_mpctraj.publish(_mpc_traj);
        }
        else
        {
            _throttle = 0.0;
            _speed = 0.0;
            _w = 0;
            if(_goal_reached && _goal_received)
            {
                cout << "Goal Reached: control loop !" << endl;
            }
        }
        // publish general cmd_vel 
        if(_pub_twist_flag)
        {
            _twist_msg.linear.x  = _speed; 
            _twist_msg.angular.z = _w;
            _pub_twist.publish(_twist_msg);
        }
    }

}