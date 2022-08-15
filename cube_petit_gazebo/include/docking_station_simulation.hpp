#ifndef DOCKING_STATION_SIMULATION
#define DOCKING_STATION_SIMULATION

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h> /* M_PI */
#include <cmath>
#include <iostream>
#include <vector>
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

class Docking_Station_Simulation
{
private:
  ros::Publisher output_volatage_pub;
  ros::Publisher output_current_pub;

  ros::Publisher robot_pose_pub, station_pose_pub;
  ros::Publisher robot_connector_pose_pub, station_connector_pose_pub;
  ros::Publisher target_pose_pub;
  ros::Publisher docking_flag_pub;

  ros::Subscriber robot_odom_sub;
  ros::Subscriber station_odom_sub;

  std::string robot_link_name;
  std::string station_link_name;

  bool publish_debug_topics_flag;

  float robot_connector_offset;
  float station_connector_offset;
  float docking_area_radious;
  float docking_area_angle;
  float voltage;
  float current;

  nav_msgs::Odometry gazebo_robot_odom;
  nav_msgs::Odometry gazebo_station_odom;

public:
  Docking_Station_Simulation(ros::NodeHandle nh);
  ~Docking_Station_Simulation();
  void stationPoseSubscriberCallback(const nav_msgs::Odometry::ConstPtr &odom);
  void robotPoseSubscriberCallback(const nav_msgs::Odometry::ConstPtr &odom);
  geometry_msgs::Quaternion yaw2quaternion(float yaw);
  void publishDataLoop();
  geometry_msgs::PoseStamped getConnectorDevitation();
  bool validateDocking(geometry_msgs::PoseStamped robot2station_connectors);
  float quaternion2yaw(geometry_msgs::Quaternion quoternion);
  Eigen::Matrix2f yaw2matrix2(float yaw);
};

#endif //DOCKING_STATION_SIMULATION