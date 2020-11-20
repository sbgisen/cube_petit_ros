#ifndef DJI_CAN_COMMUNICATION
#define DJI_CAN_COMMUNICATION

#include <cstdlib>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include "can_msgs/Frame.h"


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <regex>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <math.h>  /* M_PI */

//#include <dji_esc_driver/cansend.hpp>

class Dji_Can_Communication {
private:

  // モータID1個毎のアクセス回数
  int AccessCount[8] = {0,0,0,0,0,0,0,0};
  ros::Subscriber sub_can;

  int motor_id;
  int motor_rpm;
  double motor_current;
  int motor_temperature;
  double motor_torque;
  double motor_rad_per_sec;

  int motor_position;
  int motor_position_right;
  int motor_position_left;


  double motor_position_rad;
  double motor_rad_now;
  double motor_rad_before_right;
  double motor_rad_before_left;
  double motor_rad_before;

  // constant values, initialized in contructor
  double TORQUE_COEFFICIENT;  //トルク計数
  double REDUCTION_RATIO;   //減速比
  double MAX_CURRENT;  //C610の最大電流


  std::vector<double> rad_vec;

  enum PositionStatus {
    RAD_NOW_LEFT,
    RAD_BEFORE_LEFT,
    RAD_NOW_RIGHT,
    RAD_BEFORE_RIGHT
    };
  
  std::vector<double> status_;
  enum Params {
    POSITION_LEFT,
    VELOCITY_LEFT,
    EFFORT_LEFT,
    POSITION_RIGHT,
    VELOCITY_RIGHT,
    EFFORT_RIGHT
    };
  enum Side {
    LEFT,
    RIGHT
  };


public:
  Dji_Can_Communication();
  void initialize(ros::NodeHandle nh);
  ~Dji_Can_Communication();
  //Send_Velocity_Can(const std_msgs::String& msg);
  uint8_t can_data[8];
  // モータID1個毎のアクセス回数
  void receivedCanCallback(const can_msgs::Frame::ConstPtr& msg);
  int sendCan(uint16_t id, uint8_t* data);
  int sendVelocityCan(const double left_target_velocity, const double right_target_velocity);
  int current2Data(double current_in);
  int velocity2Data(double velocity_in);
  int createCanPacketAndSend(int right_data,int left_data);
  void updateMotorStatus(std::vector<double>&);
  // constant values, initialized in contructor
  uint8_t status_size_;
 

};



#endif //DJI_CAN_COMMUNICATION