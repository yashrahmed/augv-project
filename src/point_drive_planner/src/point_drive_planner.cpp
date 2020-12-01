
#include "point_drive_planner/point_drive_planner.h"

// pluginlib macros (defines, ...)
#include <pluginlib/class_list_macros.h>

using namespace std;

PLUGINLIB_EXPORT_CLASS(point_drive_planner::PointDrivePlannerROS, nav_core::BaseLocalPlanner)

namespace point_drive_planner
{
  const string PointDrivePlannerROS::ODOM_INPUT_TOPIC = "odom";

  PointDrivePlannerROS::PointDrivePlannerROS() : costmap_ros_(NULL), tf_buffer(NULL), initialized_(false) {}

  PointDrivePlannerROS::PointDrivePlannerROS(std::string name, tf2_ros::Buffer *tf_buffer, costmap_2d::Costmap2DROS *costmap_ros)
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
      odom_sub = gn.subscribe<nav_msgs::Odometry>(PointDrivePlannerROS::ODOM_INPUT_TOPIC, 100, &PointDrivePlannerROS::odomCallback, this);

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
      ROS_INFO("Simple Local Planner plugin initialized.");
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
    //set plan
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

    if (this->plan.size() > 2)
    {
      geometry_msgs::PoseStamped lastPose = this->plan.back();
      // @Todo -- compute turn angle and distance here....
      ROS_INFO("distance = %s", to_string(this->getDistance(lastPose)).c_str());
      ROS_INFO("turn angle = %s", to_string(this->getTurnAngle(lastPose)).c_str());



    }
    else
    {
      goal_reached_ = true;
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

  double PointDrivePlannerROS::getDistance(geometry_msgs::PoseStamped endPose)
  {
    double a_x = this->current_pose.position.x, a_y = this->current_pose.position.y;
    double b_x = endPose.pose.position.x, b_y = endPose.pose.position.y;
    return sqrt((a_x - b_x) * (a_x - b_x) + (a_y - b_y) * (a_y - b_y));
  }

  double PointDrivePlannerROS::getTurnAngle(geometry_msgs::PoseStamped endPose)
  {
    tf2::Quaternion currentQuat;
    double x1, y1, x2, y2, roll, pitch, yaw_start, dest_bearing, angle_diff, angle_sign = 1, opp_angle_diff;

    x1 = this->current_pose.position.x;
    y1 = this->current_pose.position.y;
    x2 = endPose.pose.position.x;
    y2 = endPose.pose.position.y;
    tf2::convert(this->current_pose.orientation, currentQuat);
    tf2::Matrix3x3(currentQuat).getRPY(roll, pitch, yaw_start);
    yaw_start = yaw_start >= 0 ? yaw_start : yaw_start + (2 * PI);

    dest_bearing = atan2(y2 - y1, x2 - x1);
    dest_bearing = dest_bearing >= 0 ? dest_bearing : dest_bearing + (2 * PI);

    angle_diff = dest_bearing - yaw_start;
    angle_sign = angle_diff >= 0 ? 1 : -1;
    opp_angle_diff = (2 * PI) - abs(angle_diff);

    if (opp_angle_diff < abs(angle_diff))
    {
      angle_diff = opp_angle_diff * -angle_sign;
    }

    return angle_diff;
  }

  void PointDrivePlannerROS::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    this->current_pose = msg->pose.pose;
  }
} // namespace point_drive_planner
