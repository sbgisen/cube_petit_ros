#include "jbd_battery_monitor.hpp"

////////////////////////////////////////////
//                                        //
//         jbd_battery_monitor            //
//              2020/12/25                //
//      airi.yokochi@g.softbank.co.jp     //
//                                        //
////////////////////////////////////////////

/*
Publish: 

/battery_simulation/battery_remaining
/battery_simulation/battery_voltage
/battery_simulation/charge_current

Subscribe:


Non Publish
/battery_simulation/charge_level
/battery_simulation/charge_level_mwh

*/

int main(int argc, char **argv){
  ros::init(argc, argv, "jbd_battery_monitor");
  
  ros::NodeHandle nh("~");
  ros::Publisher battery_remaining_pub = nh.advertise<std_msgs::Float64>("battery_remaining", 10);
  ros::Publisher battery_voltage_pub = nh.advertise<std_msgs::Float64>("battery_voltage", 10);
  ros::Publisher charge_current_pub = nh.advertise<std_msgs::Float64>("charge_current", 10);
  
  std_msgs::Float64 battery_remaining;
  std_msgs::Float64 battery_voltage;
  std_msgs::Float64 charge_current;

  Serial_Communication serial_comminication;
  if(serial_comminication.serial_initialize()){
    ROS_ERROR("Could Not Open Serial");
    return 1;
  }
  
  ros::Rate loop_rate(20);
  serial_comminication.updateBatteryStatus(battery_status_); 

  int count = 0;
  while(ros::ok()){
    count = 0;
    while(!serial_comminication.serial_read()){
      count++;
    }

  // 値を読み込むのは1Hz
    serial_comminication.updateBatteryStatus(battery_status_);    
    battery_remaining.data = battery_status_[BATTERY_REMAINING];
    battery_voltage.data = battery_status_[BATTERY_VOLTAGE];
    charge_current.data = battery_status_[CHARGE_CURRENT];
    battery_remaining_pub.publish(battery_remaining);
    battery_voltage_pub.publish(battery_voltage);
    charge_current_pub.publish(charge_current);

    ros::spinOnce();
    loop_rate.sleep();

  }
  return 0;
}
