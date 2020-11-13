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

public:
    Cube_Petit_Hardware_Interface();
    ~Cube_Petit_Hardware_Interface();

    Dji_Can_Communication dji_can_;

    void write();// override;  //何者
    void read();// override;



};

#endif //CUBE_PETIT_HARDWARE_INTERFACE