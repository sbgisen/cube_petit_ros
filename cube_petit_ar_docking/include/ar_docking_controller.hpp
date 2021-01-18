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
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Twist.h> // ロボットを動かすために必要


#include <linux/can.h>
#include <linux/can/raw.h>
#include <math.h>  /* M_PI */
# include "battery_current_monitor.hpp"
# include "move_base_action_client.hpp"

class AR_Docking_Controller {
private:
  ros::Publisher cmd_vel_pub;
  bool enable_go_back;

  geometry_msgs::Twist twist;

public:
  AR_Docking_Controller(ros::NodeHandle nh);
  ~AR_Docking_Controller();
  int undocking();
  int docking();
  Battery_Current_Monitor battery_current_monitor;
  Move_Base_Action_Client move_base_action_client;
  int goBack();
  double undock_sec;
  double undock_velocity;
  double undock_distance;
  bool is_docked;

};



#endif //AR_DOCKING_CONTROLLER