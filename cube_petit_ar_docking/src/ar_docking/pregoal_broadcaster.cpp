#include "pregoal_broadcaster.hpp"

/////////////////////////////
/*  コンストラクタ         */
/////////////////////////////

void Pregoal_Broadcaster::publish_pregoal(){
  // publish pregoal
  ROS_INFO("Pregoal_Broadcaster::publish_pregoal");
  // tf2_ros::StaticTransformBroadcaster static_br_;
  geometry_msgs::TransformStamped static_transformStamped;
  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "map";   //test
  static_transformStamped.child_frame_id = "pregoal";
  static_transformStamped.transform.translation.x = pregoal[0];
  static_transformStamped.transform.translation.y = pregoal[1];
  static_transformStamped.transform.translation.z = pregoal[2];
  static_transformStamped.transform.rotation.x = pregoal[3];
  static_transformStamped.transform.rotation.y = pregoal[4];
  static_transformStamped.transform.rotation.z = pregoal[5];
  static_transformStamped.transform.rotation.w = pregoal[6];
  static_br_.sendTransform(static_transformStamped);
}


Pregoal_Broadcaster::Pregoal_Broadcaster(ros::NodeHandle nh) {
  // //pregoalのROSPARAMを読み込む
  pregoal.assign(7, 0.0);
  nh.getParam("/map_info/charging_station_pose", pregoal);  //launchから起動すること
  // ROS_INFO("getparam_test %f", pregoal[1]);
  publish_pregoal();
}

Pregoal_Broadcaster::~Pregoal_Broadcaster() {

}

