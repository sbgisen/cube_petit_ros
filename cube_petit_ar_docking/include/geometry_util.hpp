#ifndef GEOMETRY_UTIL
#define GEOMETRY_UTIL

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
#include <geometry_msgs/Transform.h>

#include <geometry_msgs/Twist.h> // ロボットを動かすために必要
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>


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


class Geometry_Util {
private:

public:
  Geometry_Util();
  ~Geometry_Util();
  void setPoint(geometry_msgs::Point &point, float x, float y, float z);
  void setQuaternion(geometry_msgs::Quaternion &quatenion, float x, float y, float z, float w);
  void setPointStamped(geometry_msgs::PointStamped &point_stamped, std::string frame_name, ros::Time stamp, geometry_msgs::Point &point);
  void convertXYTheta2Pose(geometry_msgs::Pose &pose, float x, float y, float theta_rad);
  void convertXYTheta2TransformStamped(geometry_msgs::TransformStamped &trans_stamped, std::string map_frame, std::string  frame_out_name, float x, float y, float theta_rad);
  void setPoseStamped(geometry_msgs::PoseStamped &pose_stamped, std::string frame_name, ros::Time stamp, geometry_msgs::Pose &pose);
  void convertTransform2Posestamped(geometry_msgs::PoseStamped &pose_stamped, std::string frame_name, ros::Time stamp, geometry_msgs::Transform &transform);
  void convertPose2TransformStamped(geometry_msgs::TransformStamped &trans_stamped, std::string frame_name, std::string child_frame, ros::Time stamp, geometry_msgs::Pose &pose);
};



#endif //MOVE_BASE_ACTION_CLIENT