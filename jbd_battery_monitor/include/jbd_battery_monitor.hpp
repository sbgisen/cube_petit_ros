#ifndef JBD_BATTERY_MONITOR
#define JBD_BATTERY_MONITOR

#include <cstdlib>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <vector>
#include <sstream>
#include <cmath>
#include "serial_communication.hpp"


enum Params {
  BATTERY_REMAINING,
  BATTERY_VOLTAGE,
  CHARGE_CURRENT
};
std::vector<double> battery_status_(3,0.0);

#endif //JBD_BATTERY_MONITOR
