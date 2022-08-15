#ifndef PREGOAL_BROADCASTER
#define PREGOAL_BROADCASTER

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

#include <linux/can.h>
#include <linux/can/raw.h>
#include <math.h>  /* M_PI */


class Pregoal_Broadcaster {
private:
  tf2_ros::StaticTransformBroadcaster static_br_;
public:
  void publish_pregoal();
  Pregoal_Broadcaster(ros::NodeHandle nh);
  ~Pregoal_Broadcaster();
  std::vector<double> pregoal;
};



#endif //PREGOAL_BROADCASTER