#include "ar_docking_controller.hpp"


// cmd_vel_publisher
int AR_Docking_Controller::goAhead(){

  if(!enable_go_ahead){
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
  enable_go_ahead = false;
  return 0;
}




int AR_Docking_Controller::undocking(){
  ROS_INFO("AR_Docking_Controller::undocking");
  ROS_INFO("::: %d ", battery_current_monitor.is_charging());  
  enable_go_ahead = true;
  if(! goAhead()){
    is_docked = true;
    // turn in place
    return 0;
  }else{
    ROS_WARN("AR_Docking_Controller::undocking undock failed");
    return 1;
  }
}


int AR_Docking_Controller::goToPregoal(){
  ROS_INFO("AR_Docking_Controller::goToPregoal");
  //mapからpregoalの<geometry_msgs::TransformStamped>を取得する
  try{
    listener.lookupTransform("/map", "/pregoal", ros::Time(0), map2pregoal);
    is_map2pregoal = true;
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  
  //mapからpregoalのTFが存在しているか確認
  if(!is_map2pregoal){
    ROS_INFO("AR_Docking_Controller::goToPregoal TF is not found");
    return 1;
  }

  //<tf::StampedTransform>から<geometry_msgs::TransformStamped>に変換
  tf::transformTFToMsg(map2pregoal,map2pregoal_msgs);

  //<geometry_msgs::TransformStamped>から<geometry_msgs::PoseStamped>に変換
  geometry_util.convertTransform2Posestamped(poseStamped, "map", ros::Time(0),map2pregoal_msgs);
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose = poseStamped;

  // Movebaseにgoalを送信する
  int result = 0;
  result = move_base_action_client.sendGoalToActionServer(goal, 120);
  ROS_INFO("AR_Docking_Controller::goToPregoal result %d", result);
  return result;
}   

int AR_Docking_Controller::docking(){
  ROS_INFO("AR_Docking_Controller::docking");
  ROS_INFO("::: %d ", battery_current_monitor.is_charging());
  //move_base_action_client.goAhead1();
  while( goToPregoal() ){
    ROS_WARN("goToPregoal is failed...retry is 5s");
    ros::Duration(5.0).sleep();
  }

  docking_station_tf_map("ar_marker_0", "docking_station_from_map");
  goToStation("dock_start_position_0", start0_distance);


  return 0;
}

//なにをするやつ？
//"docking_station_from_map"をpublishする
double AR_Docking_Controller::docking_station_tf_map(std::string frame_in_name, std::string frame_out_name){
  ROS_INFO("AR_Docking_Controller::docking_station_tf_map");
  //map_frame
  //robot_connector_frame
  try{
    listener.waitForTransform(frame_in_name, robot_connector_frame, ros::Time(0) ,ros::Duration(10));
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  //ロボットのコネクタから見たar_marker_Xの位置
  ar_pointStamped.header.frame_id = frame_in_name;
  ar_pointStamped.header.stamp = ros::Time(0);
  ar_pointStamped.point.x = 0;
  ar_pointStamped.point.y = 0;
  ar_pointStamped.point.z = 0;
  try{
    listener.transformPoint(robot_connector_frame, ar_pointStamped, robot_to_ar_pointStamped);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  //ロボットのコネクタから見たar_marker_Xの法線(normal)の位置
  ar_normal_pointStamped.header.frame_id = frame_in_name;
  ar_normal_pointStamped.header.stamp = ros::Time(0);
  ar_normal_pointStamped.point.x = 0;
  ar_normal_pointStamped.point.y = 0;
  ar_normal_pointStamped.point.z = 1.0;
  try{
    listener.transformPoint(robot_connector_frame, ar_normal_pointStamped, robot_to_ar_normal_pointStamped);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  double ar_rotation_dx = robot_to_ar_normal_pointStamped.point.x - robot_to_ar_pointStamped.point.x;
  double ar_rotation_dy = robot_to_ar_normal_pointStamped.point.y - robot_to_ar_pointStamped.point.y;
  double ar_rotaion_theta = atan2(ar_rotation_dy, ar_rotation_dx);

  geometry_msgs::Pose tmp_pose;
  geometry_util.setPose2d(tmp_pose, robot_to_ar_pointStamped.point.x, robot_to_ar_pointStamped.point.y, ar_rotaion_theta);
  geometry_util.setPoseStamped( connector_to_station, robot_connector_frame, ros::Time(0), tmp_pose);
  
  try{
    listener.waitForTransform(frame_in_name, robot_connector_frame, ros::Time(0) ,ros::Duration(10));
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }


  //void TransformListener::transformPose	(	const std::string & 	target_frame,
  //                                        const geometry_msgs::PoseStamped & 	stamped_in,
  //                                       geometry_msgs::PoseStamped & 	stamped_out	 
  listener.transformPose(map_frame, connector_to_station, station_from_map);

  geometry_util.convertPose2TransformStamped(tf_ar, map_frame, frame_out_name, ros::Time::now(), station_from_map.pose);
  static_br_.sendTransform(tf_ar);
  return ar_rotaion_theta;
}

// 充電ドッグ"docking_station_from_map"から double distance_from_station[m]だけ指定した距離分離れた場所に移動する
// 
int AR_Docking_Controller::goToStation(std::string tf_start_position, double distance_from_station){
  ROS_INFO("AR_Docking_Controller::goToStation(%s, %f)", tf_start_position.c_str(), distance_from_station);

  std::string tf_ar = "docking_station_from_map";
  std::string tf_map = map_frame;
  // "docking_station_from_map"から"map_frame"のTFを待つ
  try{
    listener.waitForTransform(tf_ar, map_frame, ros::Time(0) ,ros::Duration(3));
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }

  geometry_msgs::PointStamped ar_origin;
  geometry_msgs::PointStamped ar_origin_on_map;
  geometry_msgs::Point tmp_point;
  geometry_util.setPoint(tmp_point, 0, 0, 0);
  geometry_util.setPointStamped(ar_origin, tf_ar, ros::Time(0), tmp_point);
  // void TransformListener::transformPoint	(	const std::string & 	target_frame,
  //                                           const geometry_msgs::PointStamped & 	stamped_in,
  //                                           geometry_msgs::PointStamped & 	stamped_out	 )			const
  listener.transformPoint(map_frame, ar_origin, ar_origin_on_map);

  ROS_INFO("origin:trans_xyz: %f, %f, %f", ar_origin_on_map.point.x, ar_origin_on_map.point.y, ar_origin_on_map.point.z);

  return 0;
}


void AR_Docking_Controller::timerCallback(const ros::TimerEvent& event){

}


/////////////////////////////
/*  コンストラクタ         */
/////////////////////////////
void AR_Docking_Controller::initialize(ros::NodeHandle nh){
  is_map2pregoal = false;
}


AR_Docking_Controller::AR_Docking_Controller(ros::NodeHandle nh):
  battery_current_monitor(nh),
  move_base_action_client(nh)
{
  enable_go_ahead = false;

  is_docked = false;
  if(battery_current_monitor.is_charging()){
    is_docked = true;
  }
  
  cmd_vel_pub= nh.advertise<geometry_msgs::Twist>("/cube_petit/diff_drive_controller/cmd_vel", 10);

  nh.getParam("/ar_docking/undock_velocity", undock_velocity);  //launchから起動すること
  nh.getParam("/ar_docking/undock_distance", undock_distance);  //launchから起動すること
  nh.getParam("/ar_docking/robot_connector_frame", robot_connector_frame);
  nh.getParam("/ar_docking/map_frame", map_frame);
  nh.getParam("/ar_docking/start0_distance",start0_distance);
  nh.getParam("/ar_docking/start1_distance",start1_distance);

  undock_sec = undock_distance/undock_velocity;  //10秒かけてアンドックする
  ROS_INFO("undock_sec is %f", undock_sec);
  timer = nh.createTimer(ros::Duration(0.5),&AR_Docking_Controller::timerCallback,this);
}

AR_Docking_Controller::~AR_Docking_Controller() {

}


