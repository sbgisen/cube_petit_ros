#include "../include/docking_station_simulation.hpp"

/////////////////////////////
/*  コンストラクタ         */
/////////////////////////////

// Docking_Station_Simulation::Docking_Station_Simulation(ros::NodeHandle nh):
//   battery_charge_simulation(nh)

Docking_Station_Simulation::Docking_Station_Simulation(ros::NodeHandle nh){
  ROS_INFO("Docking_Station_Simulation::Docking_Station_Simulation ->start");
  // get rosparam (debug)
  publish_debug_topics_flag = true;
  nh.getParam("publish_debug_topics", publish_debug_topics_flag);
  // get rosparam (link_name)
  robot_link_name = "base";
  station_link_name = "station";
  nh.getParam("robot_link_name", robot_link_name);
  nh.getParam("station_link_name", station_link_name);
  // get rosparam (docking param)
  robot_connector_offset = 0.18;
  station_connector_offset = 0.22;
  docking_area_radious = 0.03;
  docking_area_angle = 0.5;
  voltage = 30.0;
  nh.getParam("robot_connector_offset", robot_connector_offset);
  nh.getParam("station_connector_offset", station_connector_offset);
  nh.getParam("docking_area_radious", docking_area_radious);
  nh.getParam("docking_area_angle", docking_area_angle);
  nh.getParam("voltage", voltage);
  //publisher
  output_volatage_pub = nh.advertise<std_msgs::Float32>("output_voltage", 10);
  output_current_pub = nh.advertise<std_msgs::Float32>("output_current", 10);
  if(publish_debug_topics_flag){
    robot_connector_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("debug/robot_pose",5);
    station_connector_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("debug/station_pose",5);
    target_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("debug/target_pose",5);
    docking_flag_pub = nh.advertise<std_msgs::Bool>("debug/docking_flag", 5);
  }
  gazebo_robot_odom = *(ros::topic::waitForMessage<nav_msgs::Odometry>("robot_odom", nh));
  gazebo_station_odom = *(ros::topic::waitForMessage<nav_msgs::Odometry>("station_odom", nh));  
  //subscriber
  //[TODO]
  robot_odom_sub = nh.subscribe("robot_odom", 10, &Docking_Station_Simulation::robotPoseSubscriberCallback, this);
  station_odom_sub  = nh.subscribe("station_odom", 10, &Docking_Station_Simulation::stationPoseSubscriberCallback, this);
  ROS_INFO("Docking_Station_Simulation::Docking_Station_Simulation ->done");

}

Docking_Station_Simulation::~Docking_Station_Simulation() {

}


void Docking_Station_Simulation::publishDataLoop(){
  bool docking_flag = validateDocking(getConnectorDevitation());
  std_msgs::Float32 voltage_message;
  std_msgs::Float32 current_message;
  if(docking_flag){
    voltage_message.data = voltage;
    current_message.data = 5.0;
  }else{
    voltage_message.data = 0.0;
    current_message.data = -3.0;
  }
  output_current_pub.publish(current_message);
  output_volatage_pub.publish(voltage_message);
}

geometry_msgs::PoseStamped Docking_Station_Simulation::getConnectorDevitation(){
  geometry_msgs::Quaternion quaternion_world2robot;
  quaternion_world2robot.x = gazebo_robot_odom.pose.pose.orientation.x;
  quaternion_world2robot.y = gazebo_robot_odom.pose.pose.orientation.y;
  quaternion_world2robot.z = gazebo_robot_odom.pose.pose.orientation.z;
  quaternion_world2robot.w = gazebo_robot_odom.pose.pose.orientation.w;

  float yaw_world2robot = quaternion2yaw(quaternion_world2robot);  
  Eigen::Matrix2f  rotation_matrix_world2robot_vec2;
  Eigen::Matrix2f  rotation_matrix_world2robot_vec2_inv;
  rotation_matrix_world2robot_vec2 = yaw2matrix2(yaw_world2robot);
  rotation_matrix_world2robot_vec2_inv = yaw2matrix2(-yaw_world2robot);

  Eigen::Vector2f world2robot_vec2;
  Eigen::Vector2f world2station_vec2;
  world2robot_vec2(0) = gazebo_robot_odom.pose.pose.position.x;
  world2robot_vec2(1) = gazebo_robot_odom.pose.pose.position.y;
  world2station_vec2(0) = gazebo_station_odom.pose.pose.position.x;
  world2station_vec2(1) = gazebo_station_odom.pose.pose.position.y;
  
  Eigen::Vector2f robot2station_vec2;
  robot2station_vec2(0) = world2station_vec2(0) - world2robot_vec2(0);
  robot2station_vec2(1) = world2station_vec2(1) - world2robot_vec2(1);

  Eigen::Vector2f rotated_robot2station2;
  rotated_robot2station2 = rotation_matrix_world2robot_vec2_inv * robot2station_vec2;

  geometry_msgs::Quaternion quaternion_world2station;
  quaternion_world2station.x = gazebo_station_odom.pose.pose.orientation.x;
  quaternion_world2station.y = gazebo_station_odom.pose.pose.orientation.y;
  quaternion_world2station.z = gazebo_station_odom.pose.pose.orientation.z;
  quaternion_world2station.w = gazebo_station_odom.pose.pose.orientation.w;

  float yaw_world2station = quaternion2yaw(quaternion_world2station);  
  Eigen::Matrix2f rotation_matrix_world2station2;
  rotation_matrix_world2station2 = yaw2matrix2(yaw_world2station);

  float yaw_robot2station = yaw_world2station - yaw_world2robot;

  //robotのPose
  geometry_msgs::PoseStamped robot2robot;

  if(publish_debug_topics_flag){
    geometry_msgs::Quaternion quaternion_robot2station = yaw2quaternion(yaw_robot2station);

    //stationのPose
    geometry_msgs::PoseStamped robot2station;
    robot2station.header = gazebo_station_odom.header;
    robot2station.pose = gazebo_station_odom.pose.pose;
    //[謎]
    robot2station.header.frame_id = gazebo_robot_odom.child_frame_id;
    robot2station.pose.position.x = rotated_robot2station2(0);
    robot2station.pose.position.y = rotated_robot2station2(1);
    robot2station.pose.position.z = 0;
    robot2station.pose.orientation.x = quaternion_robot2station.x;
    robot2station.pose.orientation.y = quaternion_robot2station.y;
    robot2station.pose.orientation.z = quaternion_robot2station.z;
    robot2station.pose.orientation.w = quaternion_robot2station.w;

    robot2robot.header = gazebo_robot_odom.header;
    robot2robot.pose = gazebo_robot_odom.pose.pose;
    robot2robot.header.frame_id = gazebo_robot_odom.child_frame_id;
    robot2robot.pose.position.x = 0;
    robot2robot.pose.position.y = 0;
    robot2robot.pose.position.z = 0;
    robot2robot.pose.orientation.x = 0;
    robot2robot.pose.orientation.y = 0;
    robot2robot.pose.orientation.z = 0;
    robot2robot.pose.orientation.w = 1;

    robot_connector_pose_pub.publish(robot2robot);
    station_connector_pose_pub.publish(robot2station);

  }

  Eigen::Vector2f robot2connector_vec;
  robot2connector_vec(0) = robot_connector_offset;
  robot2connector_vec(1) = 0;

  Eigen::Vector2f world2robot_con_vec;
  world2robot_con_vec = world2robot_vec2 + rotation_matrix_world2robot_vec2 * robot2connector_vec;

  Eigen::Vector2f station2connector_vec;
  station2connector_vec(0) = station_connector_offset;
  station2connector_vec(1) = 0;

  Eigen::Vector2f world2station_con_vec;
  world2station_con_vec = world2station_vec2 + rotation_matrix_world2station2 * station2connector_vec;

  Eigen::Vector2f robot2station_con_vec;
  robot2station_con_vec = world2station_con_vec - world2robot_con_vec;

  Eigen::Vector2f rotated_robot2station_con_vec;
  rotated_robot2station_con_vec = rotation_matrix_world2robot_vec2_inv * robot2station_con_vec;
  // ROS_INFO("vec x: %f, y: %f",rotated_robot2station_con_vec(0),rotated_robot2station_con_vec(1));

  geometry_msgs::Quaternion quaternion_robot2target;
  quaternion_robot2target = yaw2quaternion(yaw_robot2station + M_PI);

  //targetのPose
  geometry_msgs::PoseStamped robot2station_con_pose_stamped;
  robot2robot.header = gazebo_robot_odom.header;
  robot2robot.pose = gazebo_robot_odom.pose.pose;
  robot2station_con_pose_stamped.header.frame_id = gazebo_robot_odom.child_frame_id;
  robot2station_con_pose_stamped.pose.position.x = rotated_robot2station_con_vec(0);
  robot2station_con_pose_stamped.pose.position.y = rotated_robot2station_con_vec(1);
  robot2station_con_pose_stamped.pose.position.z = 0;
  robot2station_con_pose_stamped.pose.orientation.x = quaternion_robot2target.x;
  robot2station_con_pose_stamped.pose.orientation.y = quaternion_robot2target.y;
  robot2station_con_pose_stamped.pose.orientation.z = quaternion_robot2target.z;
  robot2station_con_pose_stamped.pose.orientation.w = quaternion_robot2target.w;
  if(publish_debug_topics_flag){
    target_pose_pub.publish(robot2station_con_pose_stamped);
  }
  return robot2station_con_pose_stamped;
}

bool Docking_Station_Simulation::validateDocking(geometry_msgs::PoseStamped robot2station_connectors){
  float translation_diff = sqrt(pow(robot2station_connectors.pose.position.x,2) + pow(robot2station_connectors.pose.position.y, 2));
  
  geometry_msgs::Quaternion anguler_diff_quaternion;
  anguler_diff_quaternion.x = robot2station_connectors.pose.orientation.x;
  anguler_diff_quaternion.y = robot2station_connectors.pose.orientation.y;
  anguler_diff_quaternion.z = robot2station_connectors.pose.orientation.z;
  anguler_diff_quaternion.w = robot2station_connectors.pose.orientation.w;

  //absにintが返されるので違う気がする
  float ang_diff_temp = quaternion2yaw(anguler_diff_quaternion);
  if(ang_diff_temp < 0.0){
    ang_diff_temp = ang_diff_temp * (-1.0);
  }
  float anguler_diff = ang_diff_temp;
  bool docking_flag = false;

  // ROS_INFO("yaw: %f", quaternion2yaw(anguler_diff_quaternion));

  // ROS_INFO("trans: %f ( < %f ), ang: %f (  > %f )", translation_diff, docking_area_radious, anguler_diff, docking_area_angle);

  if( translation_diff < docking_area_radious && anguler_diff < docking_area_angle){
    docking_flag = true;
  }
  if(publish_debug_topics_flag){
    std_msgs::Bool docking_flag_msgs;
    docking_flag_msgs.data = docking_flag;
    //publish
    docking_flag_pub.publish(docking_flag_msgs);
  }

  return docking_flag;
}

float Docking_Station_Simulation::quaternion2yaw(geometry_msgs::Quaternion quoternion){
  float x = quoternion.x;
  float y = quoternion.y;
  float z = quoternion.z;
  float w = quoternion.w;
  float yaw_ret = atan2(2.0 * (w * z + x * y), w * w + x * x - y * y - z * z);
  return yaw_ret;
}

void Docking_Station_Simulation::yaw2matrix( std::vector<std::vector<float> > &rot, float yaw){
  float cos_f = cos(yaw);
  float sin_f = sin(yaw);
  float rot_ret[2][2] = {{cos_f, -sin_f}, {sin_f, cos_f}};
  rot[0][0] = cos_f;
  rot[0][1] = -sin_f;
  rot[1][0] = sin_f;
  rot[1][1] = cos_f;
}

Eigen::Matrix2f Docking_Station_Simulation::yaw2matrix2(float yaw){
  Eigen::Matrix2f rot;
  float cos_f = cos(yaw);
  float sin_f = sin(yaw);
  float rot_ret[2][2] = {{cos_f, -sin_f}, {sin_f, cos_f}};
  rot(0,0) = cos_f;
  rot(0,1) = -sin_f;
  rot(1,0) = sin_f;
  rot(1,1) = cos_f;
  return rot;
}

geometry_msgs::Quaternion Docking_Station_Simulation::yaw2quaternion(float yaw){
  geometry_msgs::Quaternion ret_quaternion;
  ret_quaternion.x = 0;
  ret_quaternion.y = 0;
  ret_quaternion.z = cos(yaw/2.0);
  ret_quaternion.w = sin(yaw/2.0);
  return ret_quaternion;
}

void Docking_Station_Simulation::robotPoseSubscriberCallback(const nav_msgs::Odometry::ConstPtr &odom_data){
  gazebo_robot_odom = *odom_data;
}

void Docking_Station_Simulation::stationPoseSubscriberCallback(const nav_msgs::Odometry::ConstPtr &odom_data){
  gazebo_station_odom = *odom_data;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "docking_statoion_simulation");

  ros::NodeHandle nh("~");

  Docking_Station_Simulation docking_station_simulation(nh);

  ros::Rate loop_rate(20);

  while(ros::ok()){
    docking_station_simulation.publishDataLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
