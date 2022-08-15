#include "geometry_util.hpp"



void Geometry_Util::setPoint(geometry_msgs::Point &point, float x, float y, float z){
  point.x = x;
  point.y = y;
  point.z = z;
}

void Geometry_Util::setQuaternion(geometry_msgs::Quaternion &quaternion, float x, float y, float z, float w){
  quaternion.x = x;
  quaternion.y = y;
  quaternion.z = z;
  quaternion.w = w;
}

void Geometry_Util::setPointStamped(geometry_msgs::PointStamped &point_stamped, std::string frame_name, ros::Time stamp, geometry_msgs::Point &point){
  point_stamped.header.frame_id = frame_name;
  point_stamped.header.stamp = stamp;
  point_stamped.point = point;
}

void Geometry_Util::convertXYTheta2Pose(geometry_msgs::Pose &pose, float x, float y, float theta_rad){
  setPoint(pose.position, x, y, 0);
  //https://qiita.com/srs/items/93d7cc671d206a07deae
  tf::Quaternion quat=tf::createQuaternionFromRPY(0, 0, theta_rad);
  geometry_msgs::Quaternion geometry_quat;
  quaternionTFToMsg(quat, geometry_quat);
  setQuaternion(pose.orientation, geometry_quat.x,geometry_quat.y,geometry_quat.z,geometry_quat.w);
}

void Geometry_Util::convertXYTheta2TransformStamped(geometry_msgs::TransformStamped &trans_stamped, std::string map_frame, std::string  frame_out_name, float x, float y, float theta_rad){
  geometry_msgs::Pose map_to_ar_pose;
  convertXYTheta2Pose(map_to_ar_pose,x,y,theta_rad);
  geometry_msgs::TransformStamped map_to_ar_tf;
  convertPose2TransformStamped(map_to_ar_tf, map_frame, frame_out_name, ros::Time(0), map_to_ar_pose);
  trans_stamped = map_to_ar_tf;
}

void Geometry_Util::setPoseStamped(geometry_msgs::PoseStamped &pose_stamped, std::string frame_name, ros::Time stamp, geometry_msgs::Pose &pose){
  pose_stamped.header.frame_id = frame_name;
  pose_stamped.header.stamp = stamp;
  pose_stamped.pose = pose;
}

void Geometry_Util::convertTransform2Posestamped(geometry_msgs::PoseStamped &pose_stamped, std::string frame_name, ros::Time stamp, geometry_msgs::Transform &transform){
  pose_stamped.header.frame_id = frame_name;
  pose_stamped.header.stamp = stamp;
  pose_stamped.pose.position.x = transform.translation.x;
  pose_stamped.pose.position.y = transform.translation.y;
  pose_stamped.pose.position.z = transform.translation.z;

  pose_stamped.pose.orientation = transform.rotation;
}

void Geometry_Util::convertPose2TransformStamped(geometry_msgs::TransformStamped &trans_stamped, std::string frame_name, std::string child_frame, ros::Time stamp, geometry_msgs::Pose &pose){
  trans_stamped.header.frame_id = frame_name;
  trans_stamped.header.stamp = stamp;
  trans_stamped.child_frame_id = child_frame;
  trans_stamped.transform.translation.x = pose.position.x;
  trans_stamped.transform.translation.y = pose.position.y;
  trans_stamped.transform.translation.z = pose.position.z;

  trans_stamped.transform.rotation = pose.orientation;
}

/////////////////////////////
/*  コンストラクタ         */
/////////////////////////////

Geometry_Util::Geometry_Util() {

}

Geometry_Util::~Geometry_Util() {

}


