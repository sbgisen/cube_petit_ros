#ifndef MOVE_BASE_ACTION_CLIENT
#define MOVE_BASE_ACTION_CLIENT

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


class Move_Base_Action_Client {
private:

public:
  Move_Base_Action_Client(ros::NodeHandle nh);
  ~Move_Base_Action_Client();
};



#endif //MOVE_BASE_ACTION_CLIENT