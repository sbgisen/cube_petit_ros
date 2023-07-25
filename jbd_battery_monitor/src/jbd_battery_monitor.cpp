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
  ros::Publisher battery_remaining_pub = nh.advertise<std_msgs::Float64>("/gazebo_battery_monitor/battery_remaining", 10);
  ros::Publisher battery_voltage_pub = nh.advertise<std_msgs::Float64>("//gazebo_battery_monitor/battery_voltage", 10);
  ros::Publisher charge_current_pub = nh.advertise<std_msgs::Float64>("/gazebo_battery_monitor/charge_current", 10);

  std_msgs::Float64 battery_remaining;
  std_msgs::Float64 battery_voltage;
  std_msgs::Float64 charge_current;

  bool once_flag = true;
  bool custom_voltage_remain = true;
  nh.getParam("/jbd_battery_monitor/once_flag", once_flag);  //launchから起動すること
  nh.getParam("/jbd_battery_monitor/custom_voltage_remain", custom_voltage_remain);

  // Batteryの MAX & MIN 電圧からバッテリの残量を算出する(BMSからのbattery_remainingが微妙なので)
  double max_voltage = 13.80;
  double min_voltage = 11.20;
  nh.getParam("/jbd_battery_monitor/max_voltage", max_voltage);
  nh.getParam("/jbd_battery_monitor/min_voltage", min_voltage);



  ROS_INFO("once_flag %d", once_flag);

  Serial_Communication serial_comminication;
  if(serial_comminication.serial_initialize()){
    ROS_ERROR("Could Not Open Serial");
    return 1;
  }

  ros::Rate loop_rate(20);
  serial_comminication.updateBatteryStatus(battery_status_);

  int count = 0;
  int loop_time = 0;
  while(ros::ok()){
    count = 0;
    while(!serial_comminication.serial_read()){
      count++;
      //continue;
    }

    // 値を読み込むのは1Hz
    serial_comminication.updateBatteryStatus(battery_status_);
    battery_remaining.data = battery_status_[BATTERY_REMAINING];
    battery_voltage.data = battery_status_[BATTERY_VOLTAGE];

    // Batteryの MAX & MIN 電圧からバッテリの残量を算出する(現在BMSからのbattery_remainingが微妙なので)
    if( custom_voltage_remain ){
        double battery_remaining_num = 100 * (battery_status_[BATTERY_VOLTAGE] - min_voltage) / (max_voltage - min_voltage);
        battery_remaining_num = std::ceil(f * 100) / 100;   //小数点2桁にする
        if (battery_remaining_num > 100.00){
            battery_remaining_num = 100.00;
        }

        battery_remaining.data = battery_remaining_num;

    }

    charge_current.data = battery_status_[CHARGE_CURRENT];
    battery_remaining_pub.publish(battery_remaining);
    battery_voltage_pub.publish(battery_voltage);
    charge_current_pub.publish(charge_current);



    ros::spinOnce();
    loop_rate.sleep();

    if(once_flag && loop_time >10){
      break;
    }
    loop_time++;

  }
  return 0;
}
