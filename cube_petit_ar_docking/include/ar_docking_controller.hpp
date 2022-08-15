#ifndef AR_DOCKING_CONTROLLER
#define AR_DOCKING_CONTROLLER

#include <stdio.h>
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
#include <time.h>
#include <chrono>

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
# include "speech_util.hpp"

#include <visualization_msgs/Marker.h>

class AR_Docking_Controller {
private:
  // Publisher
  ros::Publisher cmd_vel_pub;
  ros::Publisher connector_distance_pub;
  geometry_msgs::Twist twist;
  ros::Publisher marker_pub;
  ros::Publisher robot_heading_marker_pub, ar_heading_marker_pub, ar_nearest_marker_pub;

  // その他変数
  double undock_sec;    //アンドックする時間
  bool enable_go_ahead; //前にすすめるか・アンドック可能か
  bool is_map2pregoal;  //mapからpregoalのTFが存在しているか

  geometry_msgs::PointStamped heading_point;

  // tfMedian関数用
  int buffer_size;
  int outlier_removal_list_counter;
  double median_data[5] = {0};
  geometry_msgs::PoseStamped map_to_ar_list[5];
  double euler_list[5][3] = {{0},{0},{0},{0},{0}};

  // checkTimeout関数用
  bool timeout_counting_flag;
  time_t timeout_start_time;
  std::chrono::system_clock::time_point timeout_start_time_chrono;

  //TF
  tf2_ros::StaticTransformBroadcaster static_br_;
  tf2_ros::TransformBroadcaster dynamic_br_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_; 

  int goAhead();
  int goToPregoal();
  void initialize(ros::NodeHandle nh);
  double dockingStationTfMap(std::string frame_in_name, std::string frame_out_name);
  int goToStation(std::string tf_start_position, double distance_from_station);
  int turnInPlace(double theta_offset);
  void convertPoseStamped2MoveBaseGoal(move_base_msgs::MoveBaseGoal &goal, geometry_msgs::PoseStamped &pose_stamped);
  int lookTowardsMarker();
  int tfMedian(bool init_flag, std::string tf_in, std::string tf_out);
  int tfMedianRun(int median_times, std::string tf_in, std::string tf_out);
  void dockingStationOffset(std::string ar_frame_in, std::string ar_frame_out);
  int checkTimeout(double distance_in, bool init_flag);
  int approachStationWithCurve();
  int detectDockingFailture();
  visualization_msgs::Marker PointsToMarkerArrow(std::string frame_id_string, std::string color_string, geometry_msgs::Point start_point, geometry_msgs::Point end_point);


public:
  AR_Docking_Controller(ros::NodeHandle nh);
  ~AR_Docking_Controller();
  Battery_Current_Monitor battery_current_monitor;
  Move_Base_Action_Client move_base_action_client;
  Geometry_Util geometry_util;
  Speech_Util speech_util;

  int undocking();
  int docking();
  
  //その他変数
  bool is_docked;       //ドッキングしているか

  // getParam
  double undock_velocity;
  double undock_distance;
  double start0_distance;
  double start1_distance;
  std::string robot_connector_frame;
  std::string map_frame;
  double acceptable_angle_error;
  double station_offset_y;
  double station_offset_x;
  double curve_vel;
  double curve_robot_connector_length;
  double curve_l0;
  bool visualization_markers;

  //ros::Timer timer;
};



#endif //AR_DOCKING_CONTROLLER