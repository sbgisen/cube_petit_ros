#include "battery_charge_simulation.hpp"

///////////////////////////////////////////
/*
  battery_charge_simulation.cpp
  2021/09/16    Airi Yokochi
*/
///////////////////////////////////////////

/////////////////////////////
/*  コンストラクタ         */
/////////////////////////////

Battery_Charge_Simulation::Battery_Charge_Simulation(ros::NodeHandle nh){
  ROS_INFO("Battery_Charge_Simulation::Battery_Charge_Simulation ->start");
  // load param
  brass_battery_service_name = "/gazebo/battery_simulation/cube_petit/set_charging";
  voltage_threshold = 0.1;  
  nh.getParam("set_charging", brass_battery_service_name);
  nh.getParam("voltage_threshold", voltage_threshold);
  // docking state
  current_charging_state = false;
  docking_flag = false;

  docking_station_voltage_msg = *(ros::topic::waitForMessage<std_msgs::Float32>("docking_station_voltage_in", nh));
  voltage_sub = nh.subscribe("docking_station_voltage_in", 10, &Battery_Charge_Simulation::voltageSubscriberCallback, this);
  
  brass_gazebo_battery_client = nh.serviceClient<brass_gazebo_battery::SetCharging>(brass_battery_service_name);

  ROS_INFO("Battery_Charge_Simulation::Battery_Charge_Simulation ->done");
}

Battery_Charge_Simulation::~Battery_Charge_Simulation() {
}

// ドッキングしているか確認する：充電状況と現在のロボットの電圧を見てチャージ関数を呼び出す
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

// ロボットの電圧をSubscribeする関数
void Battery_Charge_Simulation::voltageSubscriberCallback(const std_msgs::Float32& msg){
  docking_flag = bool(msg.data > voltage_threshold);
}

// brass_gazebo_batteryのset_chargingサービス呼び出す関数
void Battery_Charge_Simulation::setChargingClient(bool charging_flag){

  ros::service::waitForService(brass_battery_service_name);  //service来るまで待つ
  brass_gazebo_battery::SetCharging srv;
  srv.request.charging = charging_flag;

  try{
    brass_gazebo_battery_client.call(srv);
    ROS_INFO("brass_gazebo_battery::SetChaginf result: %s", (srv.response.result ? "true" : "false"));
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
