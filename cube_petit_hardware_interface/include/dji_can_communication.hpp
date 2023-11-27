#ifndef DJI_CAN_COMMUNICATION
#define DJI_CAN_COMMUNICATION

#include <cstdlib>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <can_msgs/Frame.h>

#include <mutex>
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
#include <math.h> /* M_PI */

#include <control_toolbox/pid.h>
//#include <dji_esc_driver/cansend.hpp>

class DjiCanCommunication
{
private:
  // モータID1個毎のアクセス回数
  int AccessCount_[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  ros::Subscriber sub_can_;

  int motor_id_;
  int motor_rpm_;
  double motor_current_;
  int motor_temperature_;
  double motor_torque_;
  double motor_rad_per_sec_;

  int motor_position_;
  int motor_position_right_;
  int motor_position_left;

  double motor_position_rad_;
  double motor_rad_now_;
  double motor_rad_before_right_;
  double motor_rad_before_left_;
  double motor_rad_before_;

  // constant values, initialized in contructor
  double TORQUE_COEFFICIENT_;  //トルク計数
  double REDUCTION_RATIO_;     //減速比
  double MAX_CURRENT_;         // C610の最大電流

  std::vector<double> rad_vec_;

  enum PositionStatus
  {
    RAD_NOW_LEFT,
    RAD_BEFORE_LEFT,
    RAD_NOW_RIGHT,
    RAD_BEFORE_RIGHT
  };

  std::vector<double> status_;
  enum Params
  {
    POSITION_LEFT,
    VELOCITY_LEFT,
    EFFORT_LEFT,
    POSITION_RIGHT,
    VELOCITY_RIGHT,
    EFFORT_RIGHT
  };
  enum Side
  {
    LEFT,
    RIGHT
  };

public:
  DjiCanCommunication();
  void initialize(const ros::NodeHandle nh);
  ~DjiCanCommunication();
  // Send_Velocity_Can(const std_msgs::String& msg);
  uint8_t can_data_[8];
  // モータID1個毎のアクセス回数
  void receivedCanCallback(const can_msgs::Frame::ConstPtr& msg);
  int sendCan(uint16_t id, uint8_t* data);
  int sendVelocityCan(const double left_target_velocity, const double right_target_velocity);
  int current2Data(double current_in);
  int createCanPacketAndSend(int right_data, int left_data);
  void updateMotorStatus(std::vector<double>&);
  void timerCallback(const ros::TimerEvent& event);
  // constant values, initialized in contructor
  uint8_t status_size_;
  ros::Time last_time_;
  ros::Timer timer_;
  control_toolbox::Pid right_pid_;
  control_toolbox::Pid left_pid_;

  double right_target_velocity_;
  double left_target_velocity_;
  double right_target_current_;
  double left_target_current_;
};

#endif  // DJI_CAN_COMMUNICATION
