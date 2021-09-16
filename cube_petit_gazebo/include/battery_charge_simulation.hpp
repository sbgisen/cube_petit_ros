#ifndef BATTERY_CHARGE_SIMULATION
#define BATTERY_CHARGE_SIMULATION

#include <cstdlib>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
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
#include <cube_petit_gazebo/SetCharging.h>
#include <sstream>
#include <brass_gazebo_battery/SetCharging.h>


class Battery_Charge_Simulation {
private:

  std::string brass_battery_service_name;
  float docking_station_voltage;
  float voltage_threshold;
  bool current_charging_state;
  bool docking_flag;
  std_msgs::Float32 docking_station_voltage_msg;
  ros::ServiceClient brass_gazebo_battery_client;

  ros::Subscriber voltage_sub;
  
public:
  Battery_Charge_Simulation(ros::NodeHandle nh);
  ~Battery_Charge_Simulation();
  void checkDockingLoop();
  void voltageSubscriberCallback(const std_msgs::Float32& msg);
  void setChargingClient(bool charging_flag);
};



#endif //BATTERY_CHARGE_SIMULATION