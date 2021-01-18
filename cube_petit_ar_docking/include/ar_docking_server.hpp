#ifndef AR_DOCKING_SERVER
#define AR_DOCKING_SERVER

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
#include <actionlib/server/simple_action_server.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <math.h>  /* M_PI */
# include "cube_petit_ar_docking/ARDockingAction.h"
# include "pregoal_broadcaster.hpp"
# include "ar_docking_controller.hpp"
# include "battery_current_monitor.hpp"
# include "move_base_action_client.hpp"


typedef actionlib::SimpleActionServer<cube_petit_ar_docking::ARDockingAction> Server;

class AR_Docking_Server {
private:
  Server server;
  Pregoal_Broadcaster pregoal_broadcaster;
  AR_Docking_Controller ar_docking_controller;
  float current_data[10];
  int current_data_count;
  cube_petit_ar_docking::ARDockingResult result;


public:
  AR_Docking_Server(ros::NodeHandle nh);
  ~AR_Docking_Server();
  // void currentCallback(const std_msgs::Float64::ConstPtr& msg);
  // int isCharging();
  // void publish_tf_pregoal(std::vector<double>& pregoal);
  void initialize(ros::NodeHandle nh);
  void actionServerCallback(const cube_petit_ar_docking::ARDockingGoalConstPtr& goal, Server* as);
  // void actionServerCallback1(const cube_petit_ar_docking::ARDockingGoalConstPtr& goal, Server* as);
  void actionFinish(int result, Server* as);
};



#endif //AR_DOCKING_SERVER