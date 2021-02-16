#ifndef SPEECH_UTIL
#define SPEECH_UTIL

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
#include <actionlib/server/simple_action_server.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <math.h>  /* M_PI */
# include "cube_petit_ar_docking/CubeSpeech.h"

//typedef actionlib::SimpleActionServer<cube_petit_ar_docking::ARDockingAction> Server;

//typedef ros::ServiceClient<cube_petit_ar_docking::CubeSpeech> SpeechServer;

class Speech_Util {
private:
  //SpeechServer speech_server;
  cube_petit_ar_docking::CubeSpeech speech_service;
  std::string type_default_;
  std::string emotion_default_;
  int emo_level_default_;
  int pitch_default_;
  int speed_default_;
  ros::ServiceClient client;

public:
  Speech_Util(ros::NodeHandle nh);
  ~Speech_Util();
  int sayText(std::string text);
  int say(std::string text, std::string type, std::string emotion, int emo_level, int pitch, int speed);
};



#endif //SPEECH_UTIL