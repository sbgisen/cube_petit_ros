#ifndef BATTERY_CURRENT_MONITOR
#define BATTERY_CURRENT_MONITOR

#include <cstdlib>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
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

#include <linux/can.h>
#include <linux/can/raw.h>
#include <math.h>  /* M_PI */
#include <deque>
#include <iostream>

class Battery_Current_Monitor {
private:
  ros::Subscriber current_sub;
  ros::Subscriber voltage_sub;

  std::deque<double> current_deque;
  std::deque<double> voltage_deque;

  int max_queue;
  float solar_voltage_min;

public:
  Battery_Current_Monitor(ros::NodeHandle nh);
  ~Battery_Current_Monitor();
  void current_callback(const std_msgs::Float64::ConstPtr& msg);
  void voltage_callback(const std_msgs::Float32::ConstPtr& msg);
  int is_charging();
};



#endif //BATTERY_CURRENT_MONITOR