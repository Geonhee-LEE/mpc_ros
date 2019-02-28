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
    
      _pub_globalpath  = _nh.advertise<nav_msgs::Path>("/move_base/GlobalPlanner/plan", 1); // reference path for MPC ///mpc_reference 
              
      min_idx = 0;
      _pathLength = 5;
      _waypointsDist = 0;
    }

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
            {
              cout << " total_length > _pathLength" << endl;
              break;
            }
            
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
    // CallBack: Update odometry
    void GeonPlanner::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
      _odom = *odomMsg;        
    }
    void GeonPlanner::desiredPathCB(const nav_msgs::Path::ConstPtr& totalPathMsg)
    {
      _desired_path = *totalPathMsg;
    }
    
 };