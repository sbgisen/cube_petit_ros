#ifndef BATTERY_CHARGE_SIMULATION
#define BATTERY_CHARGE_SIMULATION

#include <cstdlib>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sstream>
#include <unistd.h>
#include <math.h>  /* M_PI */
#include <brass_gazebo_battery/SetCharging.h>

class Battery_Charge_Simulation {
private:
  //param
  std::string brass_battery_service_name;
  float voltage_threshold;
  
  bool current_charging_state;  //過去充電中ならtrue, 放電中ならfalse
  bool docking_flag;            //ロボットの電圧がしきい値より上であればtrue

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