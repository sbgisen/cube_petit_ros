#include "ar_docking_controller.hpp"


// cmd_vel_publisher
int AR_Docking_Controller::goBack(){

  if(!enable_go_back){
    ROS_WARN("AR_Docking_Controller::goBack() not able go back");
    //コストマップをクリアする
    ros::Duration(1).sleep();
    return 1;
  }

  for(int i=0; i< (int)undock_sec*10; i++){
    twist.linear.x = undock_velocity;
    ros::Duration(0.1).sleep();
    cmd_vel_pub.publish(twist);
  }
  for(int i=0; i< 10; i++){
    twist.linear.x = 0.0;
    ros::Duration(0.1).sleep();
    cmd_vel_pub.publish(twist);
  }
  enable_go_back = false;
  return 0;
}


int AR_Docking_Controller::undocking(){
  ROS_INFO("AR_Docking_Controller::undocking");
  ROS_INFO("::: %d ", battery_current_monitor.is_charging());  
  enable_go_back = true;
  if(! goBack()){
    is_docked = true;
    // turn in place
    return 0;
  }else{
    ROS_WARN("AR_Docking_Controller::undocking undock failed");
    return 1;
  }
}


int AR_Docking_Controller::docking(){
  ROS_INFO("AR_Docking_Controller::docking");
  ROS_INFO("::: %d ", battery_current_monitor.is_charging());
  return 0;
}


/////////////////////////////
/*  コンストラクタ         */
/////////////////////////////

AR_Docking_Controller::AR_Docking_Controller(ros::NodeHandle nh):
  battery_current_monitor(nh),
  move_base_action_client(nh)
{
  enable_go_back = false;

  is_docked = false;
  if(battery_current_monitor.is_charging()){
    is_docked = true;
  }
  
  cmd_vel_pub= nh.advertise<geometry_msgs::Twist>("/cube_petit/diff_drive_controller/cmd_vel", 10);

  nh.getParam("/ar_docking/undock_velocity", undock_velocity);  //launchから起動すること
  nh.getParam("/ar_docking/undock_distance", undock_distance);  //launchから起動すること
  undock_sec = undock_distance/undock_velocity;  //10秒かけてアンドックする
  ROS_INFO("undock_sec is %f", undock_sec);
}

AR_Docking_Controller::~AR_Docking_Controller() {

}


