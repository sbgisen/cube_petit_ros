#include "ar_docking_controller.hpp"

//-------------------------------------//
// cmd_vel_publisherで前に進む
//-------------------------------------//
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

//-------------------------------------//
// 前に進んでアンドックする
//-------------------------------------//
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

//-------------------------------------//
// ドッキング関数：0: Pregoalに行く
//-------------------------------------//
int AR_Docking_Controller::goToPregoal(){
  ROS_INFO("AR_Docking_Controller::goToPregoal");
  //mapからpregoalの<geometry_msgs::TransformStamped>を取得する
  tf::StampedTransform map2pregoal;
  geometry_msgs::TransformStamped map2pregoal_msgs;
  try{
    map2pregoal_msgs = tfBuffer_.lookupTransform("map", "pregoal", ros::Time(0));
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

  //<geometry_msgs::TransformStamped>から<geometry_msgs::PoseStamped>に変換
  geometry_msgs::PoseStamped poseStamped;
  geometry_util.convertTransform2Posestamped(poseStamped, "map", ros::Time(0),map2pregoal_msgs.transform);
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose = poseStamped;

  // Movebaseにgoalを送信する
  int result = 0;
  result = move_base_action_client.sendGoalToActionServer(goal, 120);
  ROS_INFO("AR_Docking_Controller::goToPregoal result %d", result);
  return result;
}   

//-------------------------------------//
// ドッキング関数：メイン
//-------------------------------------//
int AR_Docking_Controller::docking(){
  ROS_INFO("AR_Docking_Controller::docking");
  ROS_INFO("::: %d ", battery_current_monitor.is_charging());

  
  bool docking_success_flag = false;    //ドッキングに成功したか
  
  // 0: Pregoalに行く。成功するまで行う
  ROS_INFO("---0: go_to_pregoal------------------------------");
  while( goToPregoal() ){
    ROS_WARN("goToPregoal is failed...retry is 5s");
    ros::Duration(5.0).sleep();
  }
  sleep(1.0);
  // ARマーカーが見つかるか確認("ar_marker_0")
  try{
    tfBuffer_.lookupTransform("ar_marker_0", "base_link", ros::Time::now() ,ros::Duration(10));
  }
  catch (tf::TransformException ex){
    ROS_WARN("AR_Docking_Controller::docking(): AR Marker is not found retry...");
    ROS_ERROR("%s",ex.what());
    return 1;
  }
  // 1: ARマーカーの位置をPublishする
  ROS_INFO("---1: go to dock_start_position_0----------------");
  tfMedianRun(10, "ar_marker_0", "ar_marker_0_average");
  dockingStationTfMap("ar_marker_0_average", "ar_marker_0_on_map");
  // ARマーカーから0.5m離れた位置に移動
  goToStation("dock_start_position_0", start0_distance);

  sleep(1.0);

  // 2: もう一度ARマーカーの位置をPublishする
  ROS_INFO("---2: go to dock_start_position_1----------------");

  tfMedianRun(10, "ar_marker_0", "ar_marker_0_average");
  double angle_between_robot_and_ar_normal_vector = dockingStationTfMap("ar_marker_0_average", "ar_marker_0_on_map");
  // コネクタとARマーカーの法線の角度が規定値以上ならば失敗
  if( angle_between_robot_and_ar_normal_vector > M_PI * (acceptable_angle_error / 180) ){
    ROS_WARN("angle_between_robot_and_ar_normal_vector is %f (accept under %f ) please retry", angle_between_robot_and_ar_normal_vector , M_PI * (acceptable_angle_error / 180));
    return 1;
  }
  // ARマーカーから0.3m離れた位置に移動  
  if(goToStation("dock_start_position_1", start1_distance)){
    ROS_WARN("goToStation is failed, please retry");
    return 1;
  }
  sleep(0.2);

  // 3: 移動せずにマーカの方を見る
  ROS_INFO("---3: look towards marker ----------------");
  if(lookTowardsMarker()){
    ROS_WARN("lookTowardsMarker is failed, please retry");
    return 1;
  }
  sleep(0.8);

  // ロボットとARマーカーから垂直に出るベクトルの角度が規定値を越えてたら失敗
  angle_between_robot_and_ar_normal_vector = dockingStationTfMap("ar_marker_0", "ar_marker_0_on_map");
  if( angle_between_robot_and_ar_normal_vector > M_PI * (acceptable_angle_error / 180) ){
    ROS_WARN("angle_between_robot_and_ar_normal_vector is %f (accept under %f ) please retry", angle_between_robot_and_ar_normal_vector , M_PI * (acceptable_angle_error / 180));
    return 1;
  }

  // 4: ARマーカーの位置の平均を取得？
  // [TODO]見えなかった時に０からリスタートする
  ROS_INFO("---4: get ar marker average position"); 
  tfMedianRun(10, "ar_marker_0", "ar_marker_0_average");
  dockingStationTfMap("ar_marker_0_average", "ar_marker_0_on_map");
  // 充電ドックのコネクタの位置を"ar_average_ofs"
  dockingStationOffset("ar_marker_0_on_map", "ar_average_ofs");

  int reached_goal = battery_current_monitor.is_charging();
  int timeout = checkTimeout(0.0, true);
  int failed = 0;
  reached_goal = 0;

  int while_loop = 0;

  while(ros::ok() && docking_success_flag == false){
    while_loop++;
    ros::Duration(0.01).sleep();   //[TODO] param
    // [TODO] preempted ??

    // 充電ドッグに向かって突進する
    timeout = approachStationWithCurve();
    reached_goal = battery_current_monitor.is_charging();

    ROS_INFO("*** timeout: %d, is_charging: %d", timeout, reached_goal);

    if(!reached_goal){  failed = detectDockingFailture();}

    geometry_msgs::Twist twist_0;
    if( reached_goal && timeout == 0){
      ROS_INFO("Start Battery Charging");
      cmd_vel_pub.publish(twist_0);
      ROS_INFO("Docking succeed");
      docking_success_flag = true;
    }else if(failed){
      ROS_WARN("FAILED: Wrong docking angle detected.");
      cmd_vel_pub.publish(twist_0);
      goAhead();
      break;
    }else if(timeout == 1){
      ROS_INFO("ERR:Docking timeout.");
      cmd_vel_pub.publish(twist_0);
      goAhead();
      break;
    }


  }//while 


  enable_go_ahead = true;
  geometry_msgs::Twist twist0;
  twist0.linear.x = 0;
  twist0.angular.z = 0;
  cmd_vel_pub.publish(twist0);
  ros::param::set("/base/base_driver/duty_limiter", 0.9);
  if(docking_success_flag){
    ROS_INFO("AR_Docking_Controller::docking SUCCESS");
    return 0;
  }else{
    ROS_INFO("AR_Docking_Controller::docking FAILED");
    return 1;
  }

}

////////////////////////////////////////////////////
//
////////////////////////////////////////////////
int AR_Docking_Controller::detectDockingFailture(){
  ROS_INFO("AR_Docking_Controller::detectDockingFailture");
  std::string tf_ar="ar_average_ofs";
  std::string tf_map = map_frame;
  std::string tf_robot="connector_link";
  double detect_distance = 0.1;

  // コネクタからＡＲマーカーの中央値の中央値TFを待つ
  geometry_msgs::TransformStamped tf_robot2station;

  try{
    tf_robot2station = tfBuffer_.lookupTransform(tf_robot, tf_ar, ros::Time(0));
  }catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
  }
  geometry_msgs::PoseStamped target_from_robot;
  geometry_util.convertTransform2Posestamped(target_from_robot, tf_robot, ros::Time(0), tf_robot2station.transform);
  target_from_robot.pose.position.z = 0;
  //Quatanionからオイラー角に変換
  double euler_theta0[3] = {0.0}; //  double roll, pitch, yaw;
  tf::Quaternion quat_tmp(target_from_robot.pose.orientation.x, target_from_robot.pose.orientation.y, target_from_robot.pose.orientation.z, target_from_robot.pose.orientation.w);
  tf::Matrix3x3(quat_tmp).getRPY(euler_theta0[0],euler_theta0[1], euler_theta0[2]);

  double theta0 = euler_theta0[2] + M_PI;
  if(theta0 > M_PI){
    theta0 -= 2.0 * M_PI;
  }
  
  //rviz用
  geometry_msgs::PoseStamped connector_to_station; 
  geometry_util.setPoseStamped(connector_to_station, tf_robot, ros::Time(0), target_from_robot.pose);
  
  double distance = 0.0;
  distance = sqrt( pow(target_from_robot.pose.position.x, 2) + pow(target_from_robot.pose.orientation.y, 2) );
  ROS_INFO("  distance [%f] detect_distance [0.1]", distance);

  if(theta0 < 0.0){
    theta0 = - theta0;
  }
  ROS_INFO("  theta0 [%f] threthold [%f]", theta0, acceptable_angle_error * M_PI / 180.0);
  if(distance < detect_distance && theta0 > acceptable_angle_error * M_PI / 180.0){
    ROS_WARN("detect_distance FAILED");
    return 1;
  }
  return 0;

}

visualization_msgs::Marker AR_Docking_Controller::PointsToMarkerArrow(std::string frame_id_string, std::string color_string, geometry_msgs::Point start_point, geometry_msgs::Point end_point){
  ROS_INFO("visualization_msgs::Marker AR_Docking_Controller::PointsToMarkerArrow");

  visualization_msgs::Marker return_marker;

  return_marker.header.stamp = ros::Time::now();
  return_marker.header.frame_id = frame_id_string;
  return_marker.ns = "PointsToMarkerArrow";
  return_marker.id = 0;

  return_marker.lifetime = ros::Duration();

  return_marker.type = visualization_msgs::Marker::ARROW;
  return_marker.action = visualization_msgs::Marker::ADD;

  geometry_msgs::Vector3 arrow;  // config arrow shape
  arrow.x = 0.02;
  arrow.y = 0.04;
  arrow.z = 0.1;
  return_marker.scale = arrow;

  return_marker.points.resize(2);
  return_marker.points[0] = start_point;
  return_marker.points[1] = end_point;

  return_marker.color.r = 0.0f;
  return_marker.color.g = 0.0f;
  return_marker.color.b = 0.0f;
  return_marker.color.a = 1.0f;

  if(color_string == "b" || color_string == "blue"){
    return_marker.color.b = 1.0f;
  }else if(color_string == "r" || color_string == "red"){
    return_marker.color.r = 1.0f;
  }else if(color_string == "g" || color_string == "green"){
    return_marker.color.g = 1.0f;
  }else if(color_string == "w" || color_string == "white"){
    return_marker.color.g = 0.0f;
  }else{
    return_marker.color.r = 1.0f;
    return_marker.color.g = 1.0f;
    return_marker.color.b = 1.0f;

  }

  return return_marker;
}



////////////////////////////////////////////////////
//
////////////////////////////////////////////////
int AR_Docking_Controller::approachStationWithCurve(){
  ROS_INFO("AR_Docking_Controller::approachStationWithCurve");
  std::string tf_ar="ar_average_ofs";     //充電ドックのコネクタ位置
  std::string tf_map= map_frame;           
  std::string tf_robot="base_link_reverse"; //base_link_reverse //connector_link

  // ??
  enable_go_ahead = true;

  // コネクタからＡＲマーカーの中央値TFを待つ
  geometry_msgs::TransformStamped tf_robot2station;  // ロボットのコネクタと充電ドッグのコネクタのTF変換
  
  // コネクタからARマーカーの中央値のTF変換を取得
  try{
    tf_robot2station = tfBuffer_.lookupTransform(tf_robot, tf_ar, ros::Time(0));
  }catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
  }
  // ロボットのコネクタからARマーカーの姿勢
  geometry_msgs::PoseStamped target_from_robot;
  geometry_util.convertTransform2Posestamped(target_from_robot, tf_robot, ros::Time(0), tf_robot2station.transform);
  target_from_robot.pose.position.z = 0;
  
  //Quatanionからオイラー角に変換
  double euler_theta0[3] = {0.0}; // roll, pitch, yaw
  tf::Quaternion quat_tmp(target_from_robot.pose.orientation.x, target_from_robot.pose.orientation.y, target_from_robot.pose.orientation.z, target_from_robot.pose.orientation.w);
  tf::Matrix3x3(quat_tmp).getRPY(euler_theta0[0], euler_theta0[1], euler_theta0[2]);
  
  double distance = sqrt( pow(target_from_robot.pose.position.x, 2) + pow(target_from_robot.pose.orientation.y, 2) );
  ROS_INFO("  distance [%f]", distance);

  bool timeout_flag = checkTimeout(distance, 0);
  bool charge_flag = battery_current_monitor.is_charging();
  int reached = 0;
  if(distance < 0.5){
    ros::param::set("/base/base_driver/duty_limiter", 0.4);
  }
  geometry_msgs::PoseStamped connector_to_station;
  geometry_util.setPoseStamped(connector_to_station, tf_robot, ros::Time(0), target_from_robot.pose);
  connector_distance_pub.publish(connector_to_station);

  if(charge_flag){
    // 既にドッキングが成功している
    ROS_INFO("AR_Docking_Controller::approachStationWithCurve: already doccking success");
    reached = 0;
  }else if(timeout_flag){
    // タイムアウトした
    ROS_INFO("AR_Docking_Controller::approachStationWithCurve: Failed timeout");
    reached = 1;
  }else{
    ROS_INFO("AR_Docking_Controller::approachStationWithCurve: Go");
    double len_wheel_to_connector = curve_robot_connector_length;   //ロボットの中心からコネクタまでの距離
    double len_curve_heading = curve_l0;                            //カーブの曲率
    double velocity = curve_vel;                                    //カーブの速度


    //ROS_WARN(" target_from_robot: x[%f], y[%f]", target_from_robot.pose.position.x, target_from_robot.pose.position.y);

    geometry_msgs::PointStamped nearest_point;
    double theta_0 = M_PI + euler_theta0[2];  //M_PI
    nearest_point.point.x = target_from_robot.pose.position.x *
       pow(sin(M_PI + theta_0), 2) - target_from_robot.pose.position.y * sin(M_PI + theta_0) * cos(M_PI + theta_0);
    nearest_point.point.y = - target_from_robot.pose.position.x * 
                sin(M_PI + theta_0) * cos(M_PI + theta_0) + target_from_robot.pose.position.y * pow(cos(M_PI + theta_0), 2);
    
    ROS_INFO("  nearest point: x[%f], y[%f]", nearest_point.point.x, nearest_point.point.y);

    // double tmp_x = tan(theta_0) * ( tan(theta_0) * target_from_robot.pose.position.x - target_from_robot.pose.position.y);
    // tmp_x = tmp_x / ( tan(theta_0) * tan(theta_0) + 1);
    // double tmp_y = target_from_robot.pose.position.y - tan(theta_0) * target_from_robot.pose.position.x;
    // tmp_y = tmp_y / ( tan(theta_0) *  tan(theta_0) + 1  );

    // ROS_WARN("  nearest_point x[%f], y[%f]", tmp_x, tmp_y);

    heading_point.point.x = (nearest_point.point.x + len_curve_heading * cos(theta_0));
    heading_point.point.y = (nearest_point.point.y + len_curve_heading * sin(theta_0));
    ROS_INFO("  heading point: x[%f], y[%f]", heading_point.point.x, heading_point.point.y);

    geometry_msgs::Point point_zero;
    point_zero.x = 0;
    point_zero.y = 0;
    point_zero.z = 0;

    geometry_msgs::TransformStamped tf_robot2connector;
    try{
      tf_robot2connector = tfBuffer_.lookupTransform(tf_robot, "connector_link", ros::Time(0));
    }catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
    }
    geometry_msgs::Point connector2heading_point; //robot2connector
    connector2heading_point.x = tf_robot2connector.transform.translation.x;
    connector2heading_point.y = tf_robot2connector.transform.translation.y;
    connector2heading_point.z = 0;

    visualization_msgs::Marker marker = PointsToMarkerArrow("/base_link_reverse", "b", connector2heading_point, heading_point.point);
    robot_heading_marker_pub.publish(marker);
   
    geometry_msgs::TransformStamped tf_station2robot;
    geometry_msgs::TransformStamped tf_station2nearest;

    try{
      tf_station2robot = tfBuffer_.lookupTransform(tf_ar, tf_robot, ros::Time(0));
    }catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
    }
    tf_station2nearest = tf_station2robot;
    tf_station2nearest.transform.translation.y = 0;



    geometry_msgs::Point ar2heading;
    ar2heading.x = tf_station2robot.transform.translation.x;
    ar2heading.y = tf_station2robot.transform.translation.y;
    ar2heading.z = 0;

    geometry_msgs::Point ar2nearest;
    ar2nearest.x = tf_station2nearest.transform.translation.x;
    ar2nearest.y = tf_station2nearest.transform.translation.y;
    ar2nearest.z = 0;
    

    marker = PointsToMarkerArrow("/ar_average_ofs", "g", point_zero, ar2heading);
    ar_heading_marker_pub.publish(marker);
   
    marker = PointsToMarkerArrow("/ar_average_ofs", "r", point_zero, ar2nearest);
    ar_nearest_marker_pub.publish(marker);

    geometry_msgs::Twist target_vel;

    double connentor2heading_x = heading_point.point.x - len_wheel_to_connector;
    double connentor2heading_y = heading_point.point.y;
    double connentor2heading_length =  sqrt( pow(connentor2heading_x, 2) + pow(connentor2heading_y, 2));

    target_vel.linear.x  = - velocity * connentor2heading_x / connentor2heading_length;
    target_vel.angular.z = velocity * connentor2heading_y / connentor2heading_length / len_wheel_to_connector;

    ROS_INFO("target_vel: x[%f], z[%f]", target_vel.linear.x, target_vel.angular.z);
    cmd_vel_pub.publish(target_vel);
    reached = 2;
  }
  return reached;


}

////////////////////////////////////////////////////
//
////////////////////////////////////////////////
int AR_Docking_Controller::checkTimeout(double distance_in, bool init_flag){
  double timer_start_distance = 0.05;
  double velocity = curve_vel;
  double time_ofs = 0.5;  //0.5 5.5
  double distance_tmp = distance_in;
  
  if(init_flag){
    timeout_counting_flag = false;
    timeout_start_time_chrono = std::chrono::system_clock::now();
    return 0;
  }

  if(distance_tmp < timer_start_distance && !timeout_counting_flag ){
    ROS_INFO("Timeout countet START");
    timeout_counting_flag = true;
    timeout_start_time_chrono = std::chrono::system_clock::now();
  }
  std::chrono::system_clock::time_point time_now_tmp_chrono = std::chrono::system_clock::now();
  std::chrono::duration<float> timeout_tmp_chrono = time_now_tmp_chrono - timeout_start_time_chrono; // floatで秒を取得
  float timeout_tmp = timeout_tmp_chrono.count();
  float timeout_length = (timer_start_distance/ velocity) + time_ofs;
  if( timeout_tmp >timeout_length && timeout_counting_flag){
    return 1;
  }else{
    return 0;
  }
}

////////////////////////////////////////////////////
// ARマーカを5回見て中央値をＡＲマーカの位置として"ar_average"フレームをbroadcastする
////////////////////////////////////////////////

int AR_Docking_Controller::tfMedianRun(int median_times, std::string tf_in, std::string tf_out){

  if(tfMedian(1, tf_in, tf_out)){
    ROS_WARN("tfMedian is failed. please retry");
    return 1;
  }
  int while_roop = 0;
  while(while_roop < median_times){
    if(tfMedian(0, tf_in, tf_out)){
      ROS_WARN("tfMedian is failed. please retry");
      return 1;
    } 
    while_roop++;
  }

  return 0;
}

int AR_Docking_Controller::tfMedian(bool init_flag, std::string tf_in, std::string tf_out){
  std::string tf_ar         = tf_in;
  std::string tf_camera     = "rear_camera_optical_link";
  std::string tf_map        = map_frame;
  std::string tf_ar_average = tf_out;
  
  if(init_flag == 1){
    outlier_removal_list_counter = 0;
  }
  if(outlier_removal_list_counter > buffer_size -1){
    outlier_removal_list_counter = buffer_size - 1;
  }
  

  // "odom"からマップフレームから見たARマーカーの位置を見る
  geometry_msgs::TransformStamped tmp_ts;
  // tf::StampedTransform tmp_ts_tf;
  try{

    tfBuffer_.lookupTransform(tf_ar, tf_camera, ros::Time::now(), ros::Duration(5.0));
    tmp_ts = tfBuffer_.lookupTransform(tf_map, tf_ar, ros::Time(0));
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ROS_INFO("どうにかする");
    return 1;
  }
  // <tf::StampedTransform>型から<geometry_msgs::TransformStamped>型に変換する
  // tf::transformStampedTFToMsg(tmp_ts_tf, tmp_ts);
  map_to_ar_list[outlier_removal_list_counter].header.stamp = ros::Time::now();
  map_to_ar_list[outlier_removal_list_counter].header.frame_id = tf_map;
  map_to_ar_list[outlier_removal_list_counter].pose.position.x = tmp_ts.transform.translation.x;
  map_to_ar_list[outlier_removal_list_counter].pose.position.y = tmp_ts.transform.translation.y;
  map_to_ar_list[outlier_removal_list_counter].pose.position.z = tmp_ts.transform.translation.z;
  map_to_ar_list[outlier_removal_list_counter].pose.orientation.x = tmp_ts.transform.rotation.x;
  map_to_ar_list[outlier_removal_list_counter].pose.orientation.y = tmp_ts.transform.rotation.y;
  map_to_ar_list[outlier_removal_list_counter].pose.orientation.z =tmp_ts.transform.rotation.z;
  map_to_ar_list[outlier_removal_list_counter].pose.orientation.w =tmp_ts.transform.rotation.w;

  //Quatanionからオイラー角に変換
  double roll, pitch, yaw;
  tf::Quaternion quat_tmp(tmp_ts.transform.rotation.x, tmp_ts.transform.rotation.y, tmp_ts.transform.rotation.z, tmp_ts.transform.rotation.w);
  tf::Matrix3x3(quat_tmp).getRPY(roll, pitch, yaw);
  euler_list[outlier_removal_list_counter][0] = roll;
  euler_list[outlier_removal_list_counter][1] = pitch;
  euler_list[outlier_removal_list_counter][2] = yaw;

  int median_point = buffer_size - 1;
  if(outlier_removal_list_counter < buffer_size - 1){
    // 1から4回目の関数実行
    outlier_removal_list_counter += 1;
    return 0;
  }else{
    // 5回目の関数実行
    // median_dataにpitch角を代入
    for(int i=0; i < buffer_size; i++){
      median_data[i] = euler_list[i][2];
    }
    // median_dataの中の中央値を取得する？
    std::sort(median_data, median_data + buffer_size); 
    double median =  median_data[3];  //これは雑魚
    // 中央値に近いpitch角を保存する
    for(int i=0; i < buffer_size; i++){
      if(abs(euler_list[i][2] - median) < 0.00001){
        median_point = i;
      }
    }
    geometry_msgs::PoseStamped map_to_ar_average;
    map_to_ar_average.header.stamp = map_to_ar_list[median_point].header.stamp;
    map_to_ar_average.header.frame_id = map_to_ar_list[median_point].header.frame_id;
    map_to_ar_average.pose.position = map_to_ar_list[median_point].pose.position;
    map_to_ar_average.pose.orientation = map_to_ar_list[median_point].pose.orientation;

    geometry_msgs::TransformStamped tf_tmp;
    geometry_util.convertPose2TransformStamped(tf_tmp, tf_map, tf_ar_average, ros::Time::now(), map_to_ar_average.pose);
    static_br_.sendTransform(tf_tmp);
    outlier_removal_list_counter = 0;

  }
  return 0;
}

////////////////////////////////////////////////////
// 充電ドックのコネクタを加味して"ar_average_ofs"
// broadcast tf "ar_average_ofs"
////////////////////////////////////////////////
void AR_Docking_Controller::dockingStationOffset(std::string ar_frame_in, std::string ar_frame_out){
  ROS_INFO("AR_Docking_Controller::dockingStationOffset");
  std::string tf_map = map_frame;
  geometry_msgs::TransformStamped tf_ar_offset;
  tf_ar_offset.header.stamp = ros::Time::now();
  tf_ar_offset.header.frame_id = ar_frame_in;
  tf_ar_offset.child_frame_id = ar_frame_out;
  tf_ar_offset.transform.translation.x = station_offset_x;
  tf_ar_offset.transform.translation.y = station_offset_y;
  tf_ar_offset.transform.translation.z = 0;

  geometry_msgs::Quaternion tmp_orientation;
  tf::Quaternion quat=tf::createQuaternionFromRPY(0, 0, 0);
  geometry_msgs::Quaternion geometry_quat;
  quaternionTFToMsg(quat, geometry_quat);
  geometry_util.setQuaternion(tmp_orientation, geometry_quat.x,geometry_quat.y,geometry_quat.z,geometry_quat.w);
  tf_ar_offset.transform.rotation = tmp_orientation;
  static_br_.sendTransform(tf_ar_offset);
  try{
    bool can_transform = tfBuffer_.canTransform(tf_map, ar_frame_out, ros::Time::now() ,ros::Duration(10));
    ROS_INFO("canTransform: %d", can_transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }

}


///////////////////////////////////////////////////////////////
//"ar_marker_0_on_map"をpublishする
///////////////////////////////////////////////////////////////
double AR_Docking_Controller::dockingStationTfMap(std::string frame_in_name, std::string frame_out_name){
  geometry_msgs::PointStamped map_to_ar_pointStamped;
  geometry_msgs::PointStamped map_to_ar_normal_pointStamped;
  ROS_INFO("AR_Docking_Controller::dockingStationTfMap");

  geometry_msgs::Twist twist0;
  twist0.linear.x = 0;
  twist0.angular.z = 0;
  cmd_vel_pub.publish(twist0);


  //map_frameから見たar_marker_Xの位置
  geometry_msgs::PointStamped ar_pointStamped;
  ar_pointStamped.header.frame_id = frame_in_name;
  ar_pointStamped.header.stamp = ros::Time(0);
  ar_pointStamped.point.x = 0;
  ar_pointStamped.point.y = 0;
  ar_pointStamped.point.z = 0;

  try{
    tfBuffer_.lookupTransform(frame_in_name, map_frame, ros::Time::now() ,ros::Duration(10));
    tfBuffer_.transform(ar_pointStamped, map_to_ar_pointStamped, map_frame);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }


  //map_frameから見たar_markerの正面から1m(z+=1.0)の位置
  geometry_msgs::PointStamped ar_normal_pointStamped;
  ar_normal_pointStamped.header.frame_id = frame_in_name;
  ar_normal_pointStamped.header.stamp = ros::Time(0);
  ar_normal_pointStamped.point.x = 0;
  ar_normal_pointStamped.point.y = 0;
  ar_normal_pointStamped.point.z = 1.0;
  try{
    tfBuffer_.transform(ar_normal_pointStamped, map_to_ar_normal_pointStamped, map_frame);
   }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  // 2点の位置からmap_frameからみたＡＲマーカーの角度を計算する
  double ar_rotation_dx = map_to_ar_normal_pointStamped.point.x - map_to_ar_pointStamped.point.x;
  double ar_rotation_dy = map_to_ar_normal_pointStamped.point.y - map_to_ar_pointStamped.point.y;
  double ar_rotaion_theta = atan2(ar_rotation_dy, ar_rotation_dx);

  // map_frameからみたＡＲマーカーの姿勢をTransformStamped型に変換
  geometry_msgs::TransformStamped map_to_ar_tf;
  geometry_util.convertXYTheta2TransformStamped(map_to_ar_tf, map_frame, frame_out_name, map_to_ar_pointStamped.point.x, map_to_ar_pointStamped.point.y, ar_rotaion_theta);

  static_br_.sendTransform(map_to_ar_tf);
  return ar_rotaion_theta;
}

/////////////////////////////////////////////////////////////////////////////////////
// 充電ドッグ"ar_marker_0_on_map"から double distance_from_station[m]だけ指定した距離分離れた場所に移動する
// "dock_start_position_0", "dock_start_position_1"をPublishする
/////////////////////////////////////////////////////////////////////////////////////
int AR_Docking_Controller::goToStation(std::string tf_start_position, double distance_from_station){
  ROS_INFO("AR_Docking_Controller::goToStation(%s, %f)", tf_start_position.c_str(), distance_from_station);

  std::string tf_ar = "ar_marker_0_on_map";
  std::string tf_map = map_frame;
  // "ar_marker_0_on_map"から"map_frame"のTFを待つ
  try{
    tfBuffer_.lookupTransform(tf_ar, map_frame, ros::Time::now() ,ros::Duration(3));
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return 0;
  }

  geometry_msgs::PointStamped ar_origin;          //ARマーカの位置
  geometry_msgs::PointStamped ar_origin_on_map;   //map_frameからみたＡＲマーカの位置
  geometry_msgs::Point tmp_point;                 
  geometry_util.setPoint(tmp_point, 0, 0, 0);
  geometry_util.setPointStamped(ar_origin, tf_ar, ros::Time(0), tmp_point);
  // void TransformListener::transformPoint	(	const std::string & 	target_frame,
  //                                           const geometry_msgs::PointStamped & 	stamped_in,
  //                                           geometry_msgs::PointStamped & 	stamped_out	 )			const
  
  //map_frameからみたＡＲマーカーの位置をar_origin_on_mapに保存する
  try{
     tfBuffer_.transform(ar_origin, ar_origin_on_map, map_frame);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return 0;
  }
  ROS_INFO("origin:trans_xyz: %f, %f, %f", ar_origin_on_map.point.x, ar_origin_on_map.point.y, ar_origin_on_map.point.z);

  //ARマーカから生えるベクトル
  geometry_msgs::PointStamped ar_normal;          //ARマーカから(X+1)した位置
  geometry_msgs::PointStamped ar_normal_on_map;   //map_frameからみたＡＲマーカの正面の位置？
  geometry_util.setPoint(tmp_point, 1.0, 0, 0);
  geometry_util.setPointStamped(ar_normal, tf_ar, ros::Time(0), tmp_point);
  
  try{
    tfBuffer_.transform(ar_normal, ar_normal_on_map, map_frame);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return 0;
  }
  ROS_INFO("normal:trans_xyz: %f, %f, %f", ar_normal_on_map.point.x, ar_normal_on_map.point.y, ar_normal_on_map.point.z);

  // 
  geometry_msgs::PointStamped target_position;
  geometry_msgs::PointStamped target_on_map;   // map_frameからみた目標位置
  geometry_util.setPoint(tmp_point, distance_from_station + station_offset_x, station_offset_y, 0);
  

  geometry_util.setPointStamped(target_position, tf_ar, ros::Time(0), tmp_point);
  try{
    tfBuffer_.transform(target_position, target_on_map, map_frame);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return 0;
  }

  // 2点の位置からmap_frameからみたＡＲマーカーの角度を計算する
  double ar_rotation_dx = ar_normal_on_map.point.x - ar_origin_on_map.point.x;
  double ar_rotation_dy = ar_normal_on_map.point.y - ar_origin_on_map.point.y;
  double ar_rotaion_theta = atan2(ar_rotation_dy, ar_rotation_dx);
  ROS_INFO("diff:trans_xyz: %f, %f, %f", ar_rotation_dx, ar_rotation_dy, ar_rotaion_theta);

  //ゴール地点
  geometry_msgs::PoseStamped goal_on_map;
  geometry_msgs::Pose tmp_pose;
  geometry_util.convertXYTheta2Pose(tmp_pose, target_on_map.point.x, target_on_map.point.y, ar_rotaion_theta);
  geometry_util.setPoseStamped(goal_on_map, map_frame, ros::Time(0), tmp_pose);

  //
  // map_frameからみたゴールの姿勢をTransformStamped型に変換
  geometry_msgs::TransformStamped map_to_ar_tf;
  geometry_util.convertPose2TransformStamped(map_to_ar_tf, map_frame, tf_start_position, ros::Time::now(), goal_on_map.pose);
  static_br_.sendTransform(map_to_ar_tf);

  //
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id  = map_frame;
  goal.target_pose.header.stamp     = ros::Time::now();
  goal.target_pose.pose.position    = goal_on_map.pose.position;
  goal.target_pose.pose.orientation = goal_on_map.pose.orientation;

  // Movebaseにgoalを送信する
  int result = 0;
  result = move_base_action_client.sendGoalToActionServer(goal, 30);
  ROS_INFO("AR_Docking_Controller::goToStation result %d", result);
  return result;

}

////////////////////////////////////////
// 現在位置から移動せずマーカの方向をむく
////////////////////////////////////////
int AR_Docking_Controller::lookTowardsMarker(){
  std::string tf_ar = "ar_marker_0";
  std::string tf_robot = "base_link";

  // ロボットから見たARマーカーの位置
  geometry_msgs::PointStamped ar_origin;          //ARマーカの位置
  geometry_msgs::PointStamped ar_from_robot;      //base_linkからみたＡＲマーカの位置
  geometry_msgs::Point tmp_point;                 
  geometry_util.setPoint(tmp_point, 0.0, 0.0, 0.0);
  geometry_util.setPointStamped(ar_origin, tf_ar, ros::Time(0), tmp_point);

  try{
    tfBuffer_.lookupTransform(tf_ar, map_frame, ros::Time::now() ,ros::Duration(3));
    tfBuffer_.lookupTransform(tf_robot, map_frame, ros::Time::now() ,ros::Duration(3));
    tfBuffer_.transform(ar_origin, ar_from_robot, tf_robot);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  
  // 2点の位置からmap_frameからみたＡＲマーカーの角度を計算する
  double ar_rotation_dx = ar_from_robot.point.x;
  double ar_rotation_dy = ar_from_robot.point.y;
  double ar_rotaion_theta = atan2(ar_rotation_dy, ar_rotation_dx);
  
  geometry_msgs::PoseStamped target_from_robot;
  geometry_msgs::Pose tmp_pose;
  //[謎] 180ど回転させた
  geometry_util.convertXYTheta2Pose(tmp_pose, 0, 0, M_PI - ar_rotaion_theta);
  geometry_util.setPoseStamped(target_from_robot, tf_robot, ros::Time(0), tmp_pose);

  geometry_msgs::PoseStamped target_from_map;
  move_base_msgs::MoveBaseGoal goal;

  try{
    tfBuffer_.transform(target_from_robot, target_from_map, map_frame);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  convertPoseStamped2MoveBaseGoal(goal, target_from_map);
  int result = 0;
  result = move_base_action_client.sendGoalToActionServer(goal, 30);
  ROS_INFO("AR_Docking_Controller::lookTowardsMarker result %d", result);
  return result;
}



//<geometry_msgs::PoseStamped>型から<move_base_msgs::MoveBaseGoal>型に変換する
void AR_Docking_Controller::convertPoseStamped2MoveBaseGoal(move_base_msgs::MoveBaseGoal &goal, geometry_msgs::PoseStamped &pose_stamped){
  goal.target_pose.header.frame_id = map_frame;
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position = pose_stamped.pose.position;
  goal.target_pose.pose.position.z = 0;
  goal.target_pose.pose.orientation = pose_stamped.pose.orientation;
}


//////////////////////////////////////////////////////////////////
//その場で指定された角度[rad]回転する
//////////////////////////////////////////////////////////////////
int AR_Docking_Controller::turnInPlace(double theta_offset){
  try{
    tfBuffer_.lookupTransform("base_link", "map", ros::Time::now() ,ros::Duration(3));
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  geometry_msgs::Pose tmp_pose;
  geometry_msgs::PoseStamped target_from_robot;
  geometry_msgs::PoseStamped target_from_map;
  
  geometry_util.convertXYTheta2Pose(tmp_pose, 0, 0, theta_offset);
  geometry_util.setPoseStamped(target_from_robot, "base_link", ros::Time(0), tmp_pose);

  try{
    tfBuffer_.transform(target_from_robot, target_from_map, map_frame);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }

  // Movebaseにgoalを送信する
  move_base_msgs::MoveBaseGoal goal;
  convertPoseStamped2MoveBaseGoal(goal, target_from_map);
  int result = move_base_action_client.sendGoalToActionServer(goal, 30);
  ROS_INFO("AR_Docking_Controller::goToStation result %d", result);
  return result;

}

/////////////////////////////
/*  コンストラクタ         */
/////////////////////////////
void AR_Docking_Controller::initialize(ros::NodeHandle nh){
}


AR_Docking_Controller::AR_Docking_Controller(ros::NodeHandle nh):
  battery_current_monitor(nh),
  move_base_action_client(nh),
  speech_util(nh),
  tfBuffer_(ros::Duration(10.0)),
  tfListener_(tfBuffer_)
{
  enable_go_ahead = false;
  is_map2pregoal = false;
  buffer_size = 5;
  outlier_removal_list_counter = 0;
  
  is_docked = false;  
  if(battery_current_monitor.is_charging()){
    is_docked = true;
  }
  
  cmd_vel_pub= nh.advertise<geometry_msgs::Twist>("/cube_petit/diff_drive_controller/cmd_vel", 10);
  connector_distance_pub= nh.advertise<geometry_msgs::PoseStamped>("/distance_between_connector", 1);
  
  robot_heading_marker_pub = nh.advertise<visualization_msgs::Marker>("robot_heading_marker", 1);
  ar_heading_marker_pub = nh.advertise<visualization_msgs::Marker>("ar_heading_marker", 1);
  ar_nearest_marker_pub = nh.advertise<visualization_msgs::Marker>("ar_nearest_marker", 1);


  nh.getParam("undock_velocity", undock_velocity);  //launchから起動すること
  nh.getParam("undock_distance", undock_distance);  //launchから起動すること
  nh.getParam("robot_connector_frame", robot_connector_frame);
  nh.getParam("map_frame", map_frame);
  nh.getParam("start0_distance",start0_distance);
  nh.getParam("start1_distance",start1_distance);
  nh.getParam("acceptable_angle_error", acceptable_angle_error);
  nh.getParam("station_offset_x", station_offset_x);
  nh.getParam("station_offset_y", station_offset_y);
  nh.getParam("curve_vel", curve_vel);
  nh.getParam("curve_robot_connector_length", curve_robot_connector_length);
  nh.getParam("curve_l0", curve_l0);

  nh.getParam("visualization_markers", visualization_markers);

  undock_sec = undock_distance/undock_velocity;  //10秒かけてアンドックする
  ROS_INFO("undock_sec is %f", undock_sec);

}

AR_Docking_Controller::~AR_Docking_Controller() {

}


