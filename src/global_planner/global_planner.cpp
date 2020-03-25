 #include <pluginlib/class_list_macros.h>
 #include "global_planner.h"


 //register this planner as a BaseGlobalPlanner plugin
 PLUGINLIB_EXPORT_CLASS(global_planner::GeonPlanner, nav_core::BaseGlobalPlanner)

 using namespace std;

 //Default Constructor
 namespace global_planner 
 {
    GeonPlanner::GeonPlanner()
    {       
      cout << "Using plugin GeonPlanner"<< endl; 
        
      //Publishers and Subscribers
      _sub_odom   = _nh.subscribe("/odom", 1, &GeonPlanner::odomCB, this);
      _sub_get_path   = _nh.subscribe( "desired_path", 1, &GeonPlanner::desiredPathCB, this);   
      _sub_goal   = _nh.subscribe( "/move_base_simple/goal", 1, &GeonPlanner::goalCB, this); 
      _sub_cmd   = _nh.subscribe("/cmd_vel", 5, &GeonPlanner::getCmdCB, this);
    
      _pub_globalpath  = _nh.advertise<nav_msgs::Path>("/move_base/GlobalPlanner/plan", 1); // reference path for MPC ///mpc_reference 
              
      // Nearst point
      min_idx = 0;

      // A part of reference trajtory
      // epitrochoid,square:
      _pathLength = 3;
      // infinity:
      //_pathLength = 6;

      // Distance betweeen points of trajecotry
      _waypointsDist = 0;

      // Frequence 
      _controller_freq = 10;

      // Starting time
      _goal_received = false;

      //Save the csv file
      idx = 0;
      file.open("/home/nscl1016/catkin_ws/src/mpc_ros/pure_pursuit.csv");

      //Timer
      _errtimer = _nh.createTimer(ros::Duration((1.0)/_controller_freq), &GeonPlanner::CalError, this); // 10Hz //*****mpc

    }
    GeonPlanner::~GeonPlanner()
    {
      file.close();
    };

    GeonPlanner::GeonPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
      initialize(name, costmap_ros);
    }


    void GeonPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {

    }

    bool GeonPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan )
    {

      // Start point
      //plan.push_back(start);
      cout << " start: " <<  start.pose.position.x << endl; 
      
      nav_msgs::Path global_path = nav_msgs::Path();   // For generating mpc reference path  
      geometry_msgs::PoseStamped tempPose;
      nav_msgs::Odometry odom = _odom; 

      // middle points
      try
      {
        double total_length = 0.0;
        //find waypoints distance
              
        double gap_x = _desired_path.poses[1].pose.position.x - _desired_path.poses[0].pose.position.x;
        double gap_y = _desired_path.poses[1].pose.position.y - _desired_path.poses[0].pose.position.y;
        _waypointsDist = sqrt(gap_x*gap_x + gap_y*gap_y); 

        // Find the nearst point for robot position
        
        int min_val = 100; // why double is wrong?        
        int N = _desired_path.poses.size(); // Number of waypoints        
        const double px = odom.pose.pose.position.x; //pose: odom frame
        const double py = odom.pose.pose.position.y;
        const double ptheta = odom.pose.pose.position.y;
        
        double dx, dy; // difference distance
        double pre_yaw = 0;
        double roll, pitch, yaw = 0;

        for(int i = min_idx; i < N; i++) 
        {
            dx = _desired_path.poses[i].pose.position.x - px;
            dy = _desired_path.poses[i].pose.position.y - py;
                    
            tf::Quaternion q(
                _desired_path.poses[i].pose.orientation.x,
                _desired_path.poses[i].pose.orientation.y,
                _desired_path.poses[i].pose.orientation.z,
                _desired_path.poses[i].pose.orientation.w);
            tf::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);

            if(abs(pre_yaw - yaw) > 5)
                pre_yaw = yaw;
       
            if(min_val > sqrt(dx*dx + dy*dy) && abs(i - min_idx) < 50) 
            {
                min_val = sqrt(dx*dx + dy*dy);
                min_idx = i;
            }
        }
        for(int i = min_idx; i < N ; i++)
        {
            if(total_length > _pathLength)
              break;

            _tf_listener.transformPose("map", ros::Time(0) , 
                                            _desired_path.poses[i], "map", tempPose);                     
            global_path.poses.push_back(tempPose);                          
            total_length = total_length + _waypointsDist; 
            
            //cout << " tempPose: " <<  tempPose.pose.position.x << ", " <<  tempPose.pose.position.y << endl; 
            plan.push_back(tempPose);          
        }        
        //cout << " total_length: " <<  total_length << endl;    
        
        // Connect the end of path to the front
        if(total_length < _pathLength )
        {
            for(int i = 0; i < N ; i++)
            {
              if(total_length > _pathLength)                
                break;
              _tf_listener.transformPose("map", ros::Time(0) , 
                                                _desired_path.poses[i], "map", tempPose);                     
              global_path.poses.push_back(tempPose);                          
              total_length = total_length + _waypointsDist;  

              //cout << " tempPose: " <<  tempPose.pose.position.x << ", " <<  tempPose.pose.position.y << endl; 
              plan.push_back(tempPose);     
            }
        }  

        cout << "global_path.poses.size(): " << global_path.poses.size() << endl;
        if(global_path.poses.size() >= _pathLength )
        {
            _odom_path = global_path; // Path waypoints in odom frame
            // publish global path
            global_path.header.frame_id = "odom";
            global_path.header.stamp = ros::Time::now();
            //_pub_globalpath.publish(global_path);        
        }
        else
          cout << "Failed to path generation" << endl;

      }
      catch(tf::TransformException &ex)
      {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }     

      // Goal point
      //plan.push_back(goal);
      //cout << " goal: " <<  goal.pose.position.x << endl; 

      return true;
    }
    void GeonPlanner::CalError(const ros::TimerEvent&)
    {    
      if(_goal_received) //received goal & goal not reached    
      {   
          nav_msgs::Odometry odom = _odom; 
          nav_msgs::Path odom_path = _odom_path;   

          // Update system states: X=[x, y, theta, v]
          const double px = odom.pose.pose.position.x; //pose: odom frame
          const double py = odom.pose.pose.position.y;
          tf::Pose pose;
          tf::poseMsgToTF(odom.pose.pose, pose);
          const double theta = tf::getYaw(pose.getRotation());

          // Waypoints related parameters
          const int N = odom_path.poses.size(); // Number of waypoints
          const double costheta = cos(theta);
          const double sintheta = sin(theta);

          // Convert to the vehicle coordinate system
          Eigen::VectorXd x_veh(N);
          Eigen::VectorXd y_veh(N);
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
          const double etheta = atan(coeffs[1]);
            
          cout << "cte: " << cte << endl;
          cout << "etheta: " << etheta << endl;
          cout << "linear_vel: " << _linear_vel << endl;
          cout << "_angular_vel: " << _angular_vel << endl;
              //writefile
          idx++;
          file << idx<< "," << cte << "," <<  etheta << "," << _linear_vel << "," << _angular_vel << ",";
      }
    }
    // CallBack: Update odometry
    void GeonPlanner::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
      _odom = *odomMsg;        
    }
    void GeonPlanner::desiredPathCB(const nav_msgs::Path::ConstPtr& totalPathMsg)
    {
      _desired_path = *totalPathMsg;
    }
    // Evaluate a polynomial.
    double GeonPlanner::polyeval(Eigen::VectorXd coeffs, double x) 
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
    Eigen::VectorXd GeonPlanner::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) 
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
    // CallBack: Update goal status
    void GeonPlanner::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
    {
        _goal_received = true;
        ROS_INFO("Goal Received :goalCB!");
    }
    // CallBack: Update goal status
    void GeonPlanner::getCmdCB(const geometry_msgs::Twist& cmdMsg)
    {
      _linear_vel = cmdMsg.linear.x;
      _angular_vel = cmdMsg.angular.z;

    }

 };
