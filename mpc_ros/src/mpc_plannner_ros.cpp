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
        
        if( private_nh.getParam( "odom_frame", _odom_frame ))
        {
            odom_helper_.setOdomTopic( _odom_frame );
        }

        //Assuming this planner is being run within the navigation stack, we can
        //just do an upward search for the frequency at which its being run. This
        //also allows the frequency to be overwritten locally.

        //Parameter for topics & Frame name
        private_nh.param<std::string>("map_frame", _map_frame, "map" ); 
        private_nh.param<std::string>("odom_frame", _odom_frame, "odom");
        private_nh.param<std::string>("base_frame", _base_frame, "base_footprint");
        private_nh.param<std::string>("model_type", model_type, "unicycle");


        //Publishers and Subscribers
        _sub_odom   = _nh.subscribe("odom", 1, &MPCPlannerROS::odomCB, this);
        _pub_mpctraj   = _nh.advertise<nav_msgs::Path>("mpc_trajectory", 1);// MPC trajectory output
        _pub_odompath  = _nh.advertise<nav_msgs::Path>("mpc_reference", 1); // reference path for MPC ///mpc_reference 
        

        //Init variables
        _throttle_x = 0.0; 
        _throttle_y = 0.0;
        _w = 0.0;
        _speed_x = 0.0;
        _speed_y = 0.0;

        //_ackermann_msg = ackermann_msgs::AckermannDriveStamped();
        _twist_msg = geometry_msgs::Twist();
        _mpc_traj = nav_msgs::Path();

        
        dsrv_ = new dynamic_reconfigure::Server<MPCPlannerConfig>(private_nh);
        dynamic_reconfigure::Server<MPCPlannerConfig>::CallbackType cb = boost::bind(&MPCPlannerROS::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        initialized_ = true;
    }

  void MPCPlannerROS::reconfigureCB(MPCPlannerConfig &config, uint32_t level) {
        // update generic local planner params
        base_local_planner::LocalPlannerLimits limits;
        limits.max_vel_trans = config.max_vel_trans;
        limits.min_vel_trans = config.min_vel_trans;
        limits.max_vel_x = config.max_vel_x;
        limits.min_vel_x = config.min_vel_x;
        limits.max_vel_y = config.max_vel_y;
        limits.min_vel_y = config.min_vel_y;
        limits.max_vel_theta = config.max_vel_theta;
        limits.min_vel_theta = config.min_vel_theta;
        limits.acc_lim_x = config.acc_lim_x;
        limits.acc_lim_y = config.acc_lim_y;
        limits.acc_lim_theta = config.acc_lim_theta;
        limits.acc_lim_trans = config.acc_lim_trans;
        limits.xy_goal_tolerance = config.xy_goal_tolerance;
        limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
        limits.prune_plan = config.prune_plan;
        limits.trans_stopped_vel = config.trans_stopped_vel;
        limits.theta_stopped_vel = config.theta_stopped_vel;

        //Parameter for MPC solver
        max_vel_trans = config.max_vel_trans;
        max_vel_theta = config.max_vel_theta;
        acc_lim_trans = config.acc_lim_trans;
        _debug_info = config.debug_info;
        _delay_mode = config.delay_mode;
        _waypointsDist = config.waypoints_dist;
        _pathLength = config.path_length;
        _mpc_steps = config.steps;
        _ref_cte = config.ref_cte;
        _ref_vel = config.ref_vel;
        _ref_etheta = config.ref_etheta;
        _w_cte = config.w_cte;
        _w_etheta = config.w_etheta;
        _w_vel = config.w_vel;
        _w_angvel = config.w_angvel;
        _w_angvel_d = config.w_angvel_d;
        _w_accel_d = config.w_accel_d;
        _w_accel = config.w_accel;
        _bound_value = config.bound_value;

        ros::NodeHandle nh_move_base("~");
        nh_move_base.param("controller_frequency", controller_freq, 5.0);
        _dt = double(1.0/controller_freq); // time step duration dt in s 

        planner_util_.reconfigureCB(limits, false);

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
        _mpc_params["ANGVEL"]   = max_vel_theta;
        _mpc_params["MAXTHR"]   = acc_lim_trans;
        _mpc_params["BOUND"]    = _bound_value;
        _mpc.LoadParams(_mpc_params);
        //Display the parameters
        cout << "\n===== Parameters =====" << endl;
        cout << "debug_info: "  << _debug_info << endl;
        cout << "delay_mode: "  << _delay_mode << endl;
        //cout << "vehicle_Lf: "  << _Lf << endl;
        cout << "frequency: "   << _dt << endl;
        cout << "mpc_steps: "   << _mpc_steps << endl;
        cout << "mpc_ref_vel: " << _ref_vel << endl;
        cout << "mpc_w_cte: "   << _w_cte << endl;
        cout << "mpc_w_etheta: "  << _w_etheta << endl;
        cout << "max_vel_theta: "  << max_vel_theta << endl;

        latchedStopRotateController_.resetLatching();
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

        /*
        obstacle_costs_.setFootprint(footprint_spec);

        // costs for going away from path
        path_costs_.setTargetPoses(global_plan_);

        // costs for not going towards the local goal as much as possible
        goal_costs_.setTargetPoses(global_plan_);

        // alignment costs
        geometry_msgs::PoseStamped goal_pose = global_plan_.back();

        Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation));
        double sq_dist =
            (pos[0] - goal_pose.pose.position.x) * (pos[0] - goal_pose.pose.position.x) +
            (pos[1] - goal_pose.pose.position.y) * (pos[1] - goal_pose.pose.position.y);

        // we want the robot nose to be drawn to its final position
        // (before robot turns towards goal orientation), not the end of the
        // path for the robot center. Choosing the final position after
        // turning towards goal orientation causes instability when the
        // robot needs to make a 180 degree turn at the end
        std::vector<geometry_msgs::PoseStamped> front_global_plan = global_plan_;
        double angle_to_goal = atan2(goal_pose.pose.position.y - pos[1], goal_pose.pose.position.x - pos[0]);
        front_global_plan.back().pose.position.x = front_global_plan.back().pose.position.x +
        forward_point_distance_ * cos(angle_to_goal);
        front_global_plan.back().pose.position.y = front_global_plan.back().pose.position.y + forward_point_distance_ *
        sin(angle_to_goal);

        goal_front_costs_.setTargetPoses(front_global_plan);
        
        // keeping the nose on the path
        if (sq_dist > forward_point_distance_ * forward_point_distance_ * cheat_factor_) {
            alignment_costs_.setScale(path_distance_bias_);
        // costs for robot being aligned with path (nose on path, not ju
            alignment_costs_.setTargetPoses(global_plan_);
        } else {
        // once we are close to goal, trying to keep the nose close to anything destabilizes behavior.
            alignment_costs_.setScale(0.0);
        }
        */
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

        if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_)){
            //publish an empty plan because we've reached our goal position
            std::vector<geometry_msgs::PoseStamped> local_plan;
            std::vector<geometry_msgs::PoseStamped> transformed_plan;
            publishGlobalPlan(transformed_plan);
            publishLocalPlan(local_plan);
            base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
            ROS_WARN_NAMED("mpc_ros", "Reached the goal!!!.");
            return true;
            /*return latchedStopRotateController_.computeVelocityCommandsStopRotate(
                cmd_vel,
                limits.getAccLimits(),
                dp_->getSimPeriod(),
                &planner_util_,
                odom_helper_,
                current_pose_,
                boost::bind(&MPCPlannerROS::checkTrajectory, dp_, _1, _2, _3));*/

        }

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
        result_traj_.cost_ = 1;

        /*
        *
        *  MPC Control Loop
        * 
        */
        //copy over the odometry information
        nav_msgs::Odometry base_odom = _odom;

        // Update system states: X=[x, y, theta, v]
        const double px = base_odom.pose.pose.position.x; //pose: odom frame
        const double py = base_odom.pose.pose.position.y;
        tf::Pose pose;
        tf::poseMsgToTF(base_odom.pose.pose, pose);
        double theta = tf::getYaw(pose.getRotation());
        const double v_x = base_odom.twist.twist.linear.x; //twist: body fixed frame
        const double v_y = base_odom.twist.twist.linear.y; //twist: body fixed frame
        
        // Update system inputs: U=[w, throttle]
        const double w = _w; // steering -> w
        //const double steering = _steering;  // radian
        const double throttle_x = _throttle_x; // accel: >0; brake: <0
        const double throttle_y = _throttle_y; // accel: >0; brake: <0
        
        const double dt = _dt;

        //Update path waypoints (conversion to odom frame)
        nav_msgs::Path odom_path = nav_msgs::Path();
        try
        {
            double total_length = 0.0;
            int sampling = _downSampling;

            //find waypoints distance
            if(_waypointsDist <=0.0)
            {        
                double dx = global_plan_[1].pose.position.x - global_plan_[0].pose.position.x;
                double dy = global_plan_[1].pose.position.y - global_plan_[0].pose.position.y;
                _waypointsDist = sqrt(dx*dx + dy*dy);
                _downSampling = int(_pathLength/10.0/_waypointsDist);
            }            

            // Cut and downsampling the path
            for(int i =0; i< global_plan_.size(); i++)
            {
                if(total_length > _pathLength)
                    break;

                if(sampling == _downSampling)
                {   
                    geometry_msgs::PoseStamped tempPose;
                    tf2_ros::TransformListener tfListener(*tf_);
                    geometry_msgs::TransformStamped odom_transform;
                    odom_transform = tf_->lookupTransform(_odom_frame, _map_frame, ros::Time(0), ros::Duration(1.0) );
                    tf2::doTransform(global_plan_[i], tempPose, odom_transform); // robot_pose is the PoseStamp                     
                    odom_path.poses.push_back(tempPose);  
                    sampling = 0;
                }
                total_length = total_length + _waypointsDist; 
                sampling = sampling + 1;  
            }
           
            if(odom_path.poses.size() > 3)
            {
                // publish odom path
                odom_path.header.frame_id = "odom";
                odom_path.header.stamp = ros::Time::now();
                _pub_odompath.publish(odom_path);
            }
            else
            {
                ROS_DEBUG_NAMED("mpc_ros", "Failed to path generation since small down-sampling path.");
                _waypointsDist = -1;
                result_traj_.cost_ = -1;
                return result_traj_;
            }
            //DEBUG      
            if(_debug_info){
                cout << endl << "odom_path: " << odom_path.poses.size()
                << ", path[0]: " << odom_path.poses[0]
                << ", path[N]: " << odom_path.poses[odom_path.poses.size()-1] << endl;
            }  
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        // Waypoints related parameters
        const int N = odom_path.poses.size(); // Number of waypoints
        const double costheta = cos(theta);
        const double sintheta = sin(theta);
        
        cout << "px, py : " << px << ", "<< py << ", theta: " << theta << " , N: " << N << endl;
        // Convert to the vehicle coordinate system
        VectorXd x_veh(N);
        VectorXd y_veh(N);
        for(int i = 0; i < N; i++) 
        {
            const double dx = odom_path.poses[i].pose.position.x - px;
            const double dy = odom_path.poses[i].pose.position.y - py;
            x_veh[i] = dx * costheta + dy * sintheta;
            y_veh[i] = dy * costheta - dx * sintheta;
            //cout << "x_veh : " << x_veh[i]<< ", y_veh: " << y_veh[i] << endl;
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

        // Difference bewteen current position and goal position
        const double x_err = goal_pose.pose.position.x -  base_odom.pose.pose.position.x;
        const double y_err = goal_pose.pose.position.y -  base_odom.pose.pose.position.y;
        const double goal_err = sqrt(x_err*x_err + y_err*y_err);

        cout << "x_err:"<< x_err << ", y_err:"<< y_err  << endl;
        cout << model_type << endl;

        ros::Time begin = ros::Time::now();
        vector<double> mpc_results;

        if(model_type == "unicycle")
        {
            VectorXd state(6);
            if(_delay_mode)
            {
                // Kinematic model is used to predict vehicle state at the actual moment of control (current time + delay dt)
                const double px_act = v_x * dt;
                const double py_act = 0;
                const double theta_act = w * dt; //(steering) theta_act = v * steering * dt / Lf;
                const double vx_act = v_x + throttle_x * dt; //v = v + a * dt
                
                const double cte_act = cte + v_x * sin(etheta) * dt;
                const double etheta_act = etheta - theta_act;  
                
                state << px_act, py_act, theta_act, vx_act, cte_act, etheta_act;
            }
            else
            {
                state << 0, 0, 0, v_x, cte, etheta;
            }
            // Solve MPC Problem
            mpc_results = _mpc.unicycleModelSolve(state, coeffs); 

        }

        else if(model_type == "bicycle")
        {
            VectorXd state(6);
            if(_delay_mode)
            {
                // Kinematic model is used to predict vehicle state at the actual moment of control (current time + delay dt)
                const double px_act = v_x * dt;
                const double py_act = 0;
                const double theta_act = w * dt; //(steering) theta_act = v * steering * dt / Lf;
                const double vx_act = v_x + throttle_x * dt; //v = v + a * dt
                
                const double cte_act = cte + v_x * sin(etheta) * dt;
                const double etheta_act = etheta - theta_act;  
                
                state << px_act, py_act, theta_act, vx_act, cte_act, etheta_act;
            }
            else
            {
                state << 0, 0, 0, v_x, cte, etheta;
            }
            // Solve MPC Problem
            mpc_results = _mpc.bicycleModelSolve(state, coeffs); 

        }

        else if(model_type == "holonomic")
        {
            
            VectorXd state(7);
            if(_delay_mode)
            {
                // Kinematic model is used to predict vehicle state at the actual moment of control (current time + delay dt)
                const double px_act = v_x * dt;
                const double py_act = v_y * dt;
                const double theta_act = w * dt; //(steering) theta_act = v * steering * dt / Lf;
                const double vx_act = v_x + throttle_x * dt; //v = v + a * dt
                const double vy_act = v_y + throttle_y * dt; //v = v + a * dt
                
                //const double cte_act = cte + vx * sin(etheta) * dt + vy * cos(etheta) * dt;
                const double cte_act = cte + v_x * sin(etheta) * dt;
                const double etheta_act = etheta - theta_act;  
                
                state << px_act, py_act, theta_act, vx_act, vy_act, cte_act, etheta_act;
            }
            else
            {
                state << 0, 0, 0, v_x, v_y, cte, etheta;
            }

            // Solve MPC Problem
            mpc_results = _mpc.holonomicModelSolve(state, coeffs); 

        }

        ros::Time end = ros::Time::now();
        cout << "Duration: " << end.sec << "." << end.nsec << endl << begin.sec<< "."  << begin.nsec << endl;
            
        // MPC result (all described in car frame), output = (acceleration, w)        
        if(model_type != "holonomic")
        {
            _w = mpc_results[0]; // radian/sec, angular velocity
            _throttle_x = mpc_results[1]; // acceleration
            _throttle_y = 0; // acceleration

            _speed_x = v_x + _throttle_x * dt;  // speed
            _speed_y = 0;  // speed

            if(_speed_x >= max_vel_trans)
                _speed_x = max_vel_trans;
            if(_speed_x <= 0.0)
                _speed_x = 0.0;
        }

        else
        {
            cout << "holonomic results" << endl;

            _w = mpc_results[0]; // radian/sec, angular velocity
            _throttle_x = mpc_results[1]; // acceleration
            _throttle_y = mpc_results[2]; // acceleration

            _speed_x = v_x + _throttle_x * dt;  // speed
            _speed_y = v_y + _throttle_y * dt;  // speed
   
            cout << "thorottle_x" << _throttle_x <<endl;
            cout << "thorottle_y" << _throttle_y <<endl;
            cout << "speed_x" << _speed_x <<endl;
            cout << "speed_y" << _speed_y <<endl;


            if(_speed_x >= max_vel_trans)
                _speed_x = max_vel_trans;
            if(_speed_x < -max_vel_trans)
                _speed_x = -max_vel_trans;

            if (_speed_y >= max_vel_trans)
                _speed_y = max_vel_trans;
            if(_speed_y < -max_vel_trans)
                _speed_y = -max_vel_trans;
        }

        if(_debug_info)
        {
            cout << "\n\nDEBUG" << endl;
            cout << "theta: " << theta << endl;
            cout << "V_x: " << v_x << endl;
            cout << "V_y: " << v_y << endl;

            //cout << "odom_path: \n" << odom_path << endl;
            //cout << "x_points: \n" << x_veh << endl;
            //cout << "y_points: \n" << y_veh << endl;
            cout << "coeffs: \n" << coeffs << endl;
            cout << "_w: \n" << _w << endl;
            cout << "_throttle_x: \n" << _throttle_x << endl;
            cout << "_throttle_y: \n" << _throttle_y << endl;

            cout << "_speed_x: \n" << _speed_x << endl;
            cout << "_speed_y: \n" << _speed_y << endl;
        }

        // Display the MPC predicted trajectory
        _mpc_traj = nav_msgs::Path();
        _mpc_traj.header.frame_id = _base_frame; // points in car coordinate        
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

        if(result_traj_.cost_ < 0){
            drive_velocities.pose.position.x = 0;
            drive_velocities.pose.position.y = 0;
            drive_velocities.pose.position.z = 0;
            drive_velocities.pose.orientation.w = 1;
            drive_velocities.pose.orientation.x = 0;
            drive_velocities.pose.orientation.y = 0;
            drive_velocities.pose.orientation.z = 0;
        }

        else{
            drive_velocities.pose.position.x = _speed_x;
            drive_velocities.pose.position.y = _speed_y;
            drive_velocities.pose.position.z = 0;
            tf2::Quaternion q;
            q.setRPY(0, 0, _w);
            tf2::convert(q, drive_velocities.pose.orientation);
        }
        
        // publish the mpc trajectory
        _pub_mpctraj.publish(_mpc_traj);

        // http://docs.ros.org/en/jade/api/base_local_planner/html/classbase__local__planner_1_1SimpleTrajectoryGenerator.html#a0810ac35a39d3d7ccc1c19a862e97fbf
        // prepare cost functions and generators for this run
        /*
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
            */

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
}