
#include "point_drive_planner/point_drive_planner.h"

// pluginlib macros (defines, ...)
#include <pluginlib/class_list_macros.h>

using namespace std;

PLUGINLIB_EXPORT_CLASS(point_drive_planner::PointDrivePlannerROS, nav_core::BaseLocalPlanner)

namespace point_drive_planner
{

  PointDrivePlannerROS::PointDrivePlannerROS() : costmap_ros_(NULL), tf_buffer(NULL), initialized_(false) {}

  PointDrivePlannerROS::PointDrivePlannerROS(std::string name, tf2_ros::Buffer *tf_buffer , costmap_2d::Costmap2DROS *costmap_ros)
      : costmap_ros_(NULL), tf_buffer(NULL), initialized_(false)
  {
    // initialize planner
    initialize(name, tf_buffer, costmap_ros);
  }

  PointDrivePlannerROS::~PointDrivePlannerROS() {}

  void PointDrivePlannerROS::initialize(std::string name, tf2_ros::Buffer *tf_buffer, costmap_2d::Costmap2DROS *costmap_ros)
  {

    // check if the plugin is already initialized
    if (!initialized_)
    {
      // copy adress of costmap and Transform Listener (handed over from move_base)
      costmap_ros_ = costmap_ros;
      this->tf_buffer = tf_buffer;

      // subscribe to topics (to get odometry information, we need to get a handle to the topic in the global namespace)
      ros::NodeHandle gn;
      path_pub = gn.advertise<visualization_msgs::Marker>("visualization_marker", 10);

      //initializing the visualization markers
      points.header.frame_id = "/map";
      points.header.stamp = ros::Time::now();
      points.ns = "path_drawing";
      points.action = visualization_msgs::Marker::ADD;
      points.id = 0;
      points.type = visualization_msgs::Marker::POINTS;
      points.scale.x = 0.1;
      points.scale.y = 0.1;
      points.color.g = 1.0f;
      points.color.a = 1.0;

      
      // set initialized flag
      initialized_ = true;

      // this is only here to make this process visible in the rxlogger right from the start
      ROS_DEBUG("Simple Local Planner plugin initialized.");
    }
    else
    {
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
  }

  bool PointDrivePlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
  {

    // check if plugin initialized
    if (!initialized_)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }


    //set plan, length and next goal
    plan = orig_global_plan;

    // set goal as not reached
    goal_reached_ = false;

    return true;
  }

  bool PointDrivePlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
  {

    // check if plugin initialized
    if (!initialized_)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    return true;
  }

  bool PointDrivePlannerROS::isGoalReached()
  {
    // check if plugin initialized
    if (!initialized_)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    // this info comes from compute velocity commands:
    return goal_reached_;
  }

  void PointDrivePlannerROS::pathVisualization()
  {

    geometry_msgs::Point p;

    points.points.push_back(p);

    path_pub.publish(points);
  }

} // namespace point_drive_planner