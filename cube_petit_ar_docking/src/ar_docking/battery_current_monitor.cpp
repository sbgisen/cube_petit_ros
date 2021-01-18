#include "battery_current_monitor.hpp"

/////////////////////////////
/*  コンストラクタ         */
/////////////////////////////

Battery_Current_Monitor::Battery_Current_Monitor(ros::NodeHandle nh) {
  ROS_INFO("Battery_Current_Monitor::Battery_Current_Monitor");
  //current_sub = nh.subscribe<std_msgs::Float64>("/gazebo_battery_monitor/charge_current", 1, &Battery_Current_Monitor::current_callback, this);
  voltage_sub = nh.subscribe<std_msgs::Float32>("/solar_voltage", 1, &Battery_Current_Monitor::voltage_callback, this);

  max_queue = 10;//電流値を保存するキューは10個
  solar_voltage_min = 12.0;
  current_deque.assign(10,-1.0);
}

Battery_Current_Monitor::~Battery_Current_Monitor() {
}

void Battery_Current_Monitor::current_callback(const std_msgs::Float64::ConstPtr& msg){
  if(current_deque.size() >= max_queue){
    //先頭を取り出す
    current_deque.pop_front();
  }
  current_deque.push_back(msg->data);
}


void Battery_Current_Monitor::voltage_callback(const std_msgs::Float32::ConstPtr& msg){
  if(voltage_deque.size() >= max_queue){
    voltage_deque.pop_front();    //先頭を取り出す
  }
  voltage_deque.push_back(msg->data);
}



// // 充電できているか判定する
int Battery_Current_Monitor::is_charging(){
  int is_charging = 0;
  ROS_INFO("Battery_Current_Monitor::is_charging");
  // if(!current_deque.empty()){
  //   for(int i=0; i<current_deque.size(); i++){
  //     if(current_deque[i] <= 0.0){
  //       is_charging = 0;
  //       return is_charging;
  //     }
  //   }
  //   is_charging = 1;
  // }

  if(!voltage_deque.empty()){
    for(int i=0; i<voltage_deque.size(); i++){
      if(voltage_deque[i] <= solar_voltage_min){
        is_charging = 0;
        return is_charging;
      }
    }
    is_charging = 1;
  }

  return is_charging;
}
