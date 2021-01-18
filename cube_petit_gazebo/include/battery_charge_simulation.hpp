#ifndef BATTERY_CHARGE_SIMULATION
#define BATTERY_CHARGE_SIMULATION

#include <cstdlib>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
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
#include <math.h>  /* M_PI */

class Battery_Charge_Simulation {
private:

public:
  Battery_Charge_Simulation();
  ~Battery_Charge_Simulation();
};



#endif //BATTERY_CHARGE_SIMULATION