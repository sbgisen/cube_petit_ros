#ifndef CUBE_PETIT_HARDWARE_INTERFACE
#define CUBE_PETIT_HARDWARE_INTERFACE

#include <cstdlib>
#include <ros/ros.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>

#include "dji_can_communication.hpp"
#include <std_msgs/String.h>
#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <vector>


class Cube_Petit_Hardware_Interface : public hardware_interface::RobotHW{

private:
  //std::shared_ptr<Dji_Can_Communication> dji_can_ptr_;
  Dji_Can_Communication dji_can_;
  hardware_interface::JointStateInterface    joint_state_interface_;
  hardware_interface::VelocityJointInterface joint_velocity_interface_;

  std::vector<uint8_t> mode_;
  std::vector<double> position_, velocity_, effort_;
  std::vector<double> velocity_command_;
  std::vector<bool> direction_;
  std::vector<double> status_;

  enum Params {
    POSITION_LEFT,
    VELOCITY_LEFT,
    EFFORT_LEFT,
    POSITION_RIGHT,
    VELOCITY_RIGHT,
    EFFORT_RIGHT
  };

  //モータの左右
  enum Side {
    LEFT,
    RIGHT
  };

public:
  Cube_Petit_Hardware_Interface();
  ~Cube_Petit_Hardware_Interface();
  void initialize(ros::NodeHandle nh);
  std::string execCmd(std::string system_cmd);
  void update();
  void write();// override;  //いらない
  void read();// override;
  ros::Time getTime();
  ros::Duration getPeriod();
};

#endif //CUBE_PETIT_HARDWARE_INTERFACE