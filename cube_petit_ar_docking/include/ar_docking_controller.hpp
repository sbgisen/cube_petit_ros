#ifndef AR_DOCKING_CONTROLLER
#define AR_DOCKING_CONTROLLER

#include <cstdlib>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <mutex>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <regex>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <geometry_msgs/Twist.h> // ロボットを動かすために必要
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>


#include <linux/can.h>
#include <linux/can/raw.h>
#include <math.h>  /* M_PI */
# include "battery_current_monitor.hpp"
# include "move_base_action_client.hpp"
# include "geometry_util.hpp"

class AR_Docking_Controller {
private:
  ros::Publisher cmd_vel_pub;
  geometry_msgs::Twist twist;

public:
  AR_Docking_Controller(ros::NodeHandle nh);
  ~AR_Docking_Controller();
  Battery_Current_Monitor battery_current_monitor;
  Move_Base_Action_Client move_base_action_client;
  Geometry_Util geometry_util;

  int undocking();
  int docking();
  int goAhead();
  int goToPregoal();
  bool get_tf(geometry_msgs::TransformStamped& tf, const std::string parent_frame, const std::string child_frame, const float timeout);
  void timerCallback(const ros::TimerEvent& event);
  void initialize(ros::NodeHandle nh);
  double docking_station_tf_map(std::string frame_in_name, std::string frame_out_name);
  int goToStation(std::string tf_start_position, double distance_from_station);

  double undock_sec;

  bool is_docked;
  bool enable_go_ahead;
  bool is_map2pregoal;

  geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::PoseStamped poseStamped;
  geometry_msgs::PoseStamped ar_poseStamped;
  geometry_msgs::PointStamped ar_pointStamped;
  geometry_msgs::PoseStamped ar_normal_poseStamped;
  geometry_msgs::PointStamped ar_normal_pointStamped;
  geometry_msgs::PoseStamped connector_to_station;
  geometry_msgs::PoseStamped station_from_map;

  ros::Timer timer;
  tf::TransformListener listener;
  tf2_ros::StaticTransformBroadcaster static_br_;
  tf2_ros::TransformBroadcaster dynamic_br_;

  tf::StampedTransform map2pregoal;
  geometry_msgs::Transform map2pregoal_msgs;

  tf::StampedTransform map2connector;

  // getParam
  double undock_velocity;
  double undock_distance;
  double start0_distance;
  double start1_distance;
  std::string robot_connector_frame;
  std::string map_frame;
};



#endif //AR_DOCKING_CONTROLLER