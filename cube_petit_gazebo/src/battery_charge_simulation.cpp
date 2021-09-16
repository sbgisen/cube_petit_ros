#include "../include/battery_charge_simulation.hpp"


/////////////////////////////
/*  コンストラクタ         */
/////////////////////////////

Battery_Charge_Simulation::Battery_Charge_Simulation(ros::NodeHandle nh){
  ROS_INFO("Battery_Charge_Simulation::Battery_Charge_Simulation ->start");

  ros::Rate loop_rate(20);
  
  docking_station_voltage_msg = *(ros::topic::waitForMessage<std_msgs::Float32>("docking_station_voltage_in", nh));

  brass_battery_service_name = "/gazebo/battery_simulation/cube_petit/set_charging";
  voltage_threshold = 0.1;
  current_charging_state = false;
  docking_flag = false;
  
  nh.getParam("set_charging", brass_battery_service_name);
  nh.getParam("voltage_threshold", voltage_threshold);

  ROS_INFO("------brass_battery_service_name: %s", brass_battery_service_name.c_str());
  
  voltage_sub = nh.subscribe("docking_station_voltage_in", 10, &Battery_Charge_Simulation::voltageSubscriberCallback, this);
  brass_gazebo_battery_client = nh.serviceClient<brass_gazebo_battery::SetCharging>(brass_battery_service_name);

  ROS_INFO("Battery_Charge_Simulation::Battery_Charge_Simulation ->done");
}

Battery_Charge_Simulation::~Battery_Charge_Simulation() {

}

void Battery_Charge_Simulation::checkDockingLoop(){
  if(docking_flag && !current_charging_state){
    ROS_INFO("brass_gazebo_battery::checkDockingLoop -> true");
    setChargingClient(true);
    current_charging_state = true;
  }else if (!docking_flag && current_charging_state){
    ROS_INFO("brass_gazebo_battery::checkDockingLoop -> false");
    setChargingClient(false);
    current_charging_state = false;
  }
}

void Battery_Charge_Simulation::voltageSubscriberCallback(const std_msgs::Float32& msg){
  docking_flag = bool(msg.data > voltage_threshold);
}

void Battery_Charge_Simulation::setChargingClient(bool charging_flag){
  //service来るまで待つ
  ros::service::waitForService(brass_battery_service_name);

  brass_gazebo_battery::SetCharging srv;
  srv.request.charging = charging_flag;

  try{
    brass_gazebo_battery_client.call(srv);
    ROS_INFO("brass_gazebo_battery::SetChaginf result:");// %f", srv.response.result);
  }catch(ros::Exception ex){
    ROS_ERROR("%s",ex.what());
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "battery_charge_simulation");
  ros::NodeHandle nh("~");

  Battery_Charge_Simulation battery_charge_simulation(nh);

  ros::Rate loop_rate(20);

  while(ros::ok()){
    battery_charge_simulation.checkDockingLoop();
    ros::spinOnce();
    loop_rate.sleep();

  }
  return 0;
}
