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

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <math.h>  /* M_PI */

//#include <dji_esc_driver/cansend.hpp>

class Dji_Can_Communication {
private:
  // データ
  int Current_16[8] = {0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000};
  // モータID1個毎のアクセス回数
  int AccessCount[8] = {0,0,0,0,0,0,0,0};
  ros::Subscriber sub_can;

  int motor_id;
  int motor_degree_now;
  int motor_degree_before;
  int motor_rpm;
  int motor_current;
  int motor_temperature;
  double motor_torque;
  double motor_rad_per_sec;
  int motor_position;

  // constant values, initialized in contructor
  double TORQUE_COEFFICIENT = 0.18;  //トルク計数
  double REDUCTION_RATIO = 36;   //減速比
  int MAX_CURRENT = 10000;  //C610の最大電流


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
  void received_can_callback(const can_msgs::Frame::ConstPtr& msg);
  int send_Can(uint16_t id, uint8_t* data);
  int send_Velocity_Can(const double left_target_velocity, const double right_target_velocity);
  int export_Velocity_To_Can(int id, double target_velocity);
  void updateMotorStatus(std::vector<double>&);
  void loop_ros();
  // constant values, initialized in contructor
  uint8_t status_size_;
  int velocityToCurrent(double velocity);
  

};



#endif //DJI_CAN_COMMUNICATION