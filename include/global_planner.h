 /** include the libraries you need in your planner here */
 /** for global path planner interface */
 #include <ros/ros.h>
 #include <costmap_2d/costmap_2d_ros.h>
 #include <costmap_2d/costmap_2d.h>
 #include <nav_core/base_global_planner.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <angles/angles.h>
 #include <base_local_planner/world_model.h>
 #include <base_local_planner/costmap_model.h>

 #include <nav_msgs/Odometry.h>
 #include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

#include <Eigen/Core>
#include <Eigen/QR>
#include <vector>
#include <map>

// inlcude iostream and string libraries
#include <iostream>
#include <fstream>
#include <string>

 using std::string;

 #ifndef GLOBAL_PLANNER_CPP
 #define GLOBAL_PLANNER_CPP

 namespace global_planner {

 class GeonPlanner : public nav_core::BaseGlobalPlanner {
 public:
    GeonPlanner();
    ~GeonPlanner();
    
    GeonPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /** overridden classes from interface nav_core::BaseGlobalPlanner **/
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan
               );
  
    ros::NodeHandle _nh;
    ros::Timer _errtimer;
    ros::Publisher _pub_globalpath;
    ros::Subscriber _sub_odom, _sub_get_path, _sub_goal, _sub_cmd;
    nav_msgs::Odometry _odom;
    nav_msgs::Path _odom_path;
    nav_msgs::Path _desired_path;
    tf::TransformListener _tf_listener;

    double _waypointsDist;  //minimum distance between points of path
    int min_idx; //nearest point
    double _pathLength;
    double _cte;
    double _oreient;
    int _controller_freq;
    bool _goal_received;
    double _linear_vel;
    double _angular_vel;

    unsigned int idx;
    std::fstream file;

    void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
    void desiredPathCB(const nav_msgs::Path::ConstPtr& pathMsg);
    void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
    void CalError(const ros::TimerEvent&);
    void getCmdCB(const geometry_msgs::Twist&);

    double polyeval(Eigen::VectorXd coeffs, double x);        
    Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);


  };
 };
 #endif

