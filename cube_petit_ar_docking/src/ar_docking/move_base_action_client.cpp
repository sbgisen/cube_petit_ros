#include "move_base_action_client.hpp"


/////////////////////////////
/*  コンストラクタ         */
/////////////////////////////

enum Move_Base_Result_Params{
  mvSUCCEED,
  mvABORTED,
  mvPREENMTED
};


//PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST


// for test
void Move_Base_Action_Client::goAhead1(){
  goal_.target_pose.header.frame_id = "base_link";
  goal_.target_pose.header.stamp = ros::Time::now();
  goal_.target_pose.pose.position.x = 1.0;
  goal_.target_pose.pose.orientation.w = 1.0;
  ac.sendGoal(goal_);
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");
}


int Move_Base_Action_Client::setAndSendGoal(move_base_msgs::MoveBaseGoal goal, int duration){
  ROS_INFO("Move_Base_Action_Client::setAndSendGoal");

  goal_.target_pose.header.frame_id = goal.target_pose.header.frame_id;
  goal_.target_pose.header.stamp = ros::Time::now();
  goal_.target_pose.pose.position = goal.target_pose.pose.position;
  goal_.target_pose.pose.position.z = 0;
  goal_.target_pose.pose.orientation = goal.target_pose.pose.orientation;

  ac.sendGoal(goal_);
  bool result = ac.waitForResult(ros::Duration(duration));
  actionlib::SimpleClientGoalState status = ac.getState();
  if(result && status == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Move_Base_Action_Client::setAndSendGoal: succeeded");
    return 0;
  }else{
    ROS_INFO("Move_Base_Action_Client::setAndSendGoal: failed");
    return 1;
  }
}

int Move_Base_Action_Client::sendGoalToActionServer(move_base_msgs::MoveBaseGoal goal, int duration){
  ROS_INFO("Move_Base_Action_Client::sendGoalToActionServer");
  int result = setAndSendGoal(goal, duration);
  return result;
}

// コストマップをクリアする
void Move_Base_Action_Client::clearCostmap(){
  std_srvs::Empty cl_cm;
  if(clear_costmap->call(cl_cm)){
    ROS_WARN("move_base:Costmaps have been cleared!");
  }else{
    ROS_WARN("move_base:Fail to clear costmaps!");
  }
}

void Move_Base_Action_Client::cancelGoals(){
  ac.cancelAllGoals();
}

// 適当・いらん
int Move_Base_Action_Client::getResult(){
  if(move_base_final_state==actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Move_Base_Action_Client::getResult:Succeed 0 ");
    return 0;
  }else if(move_base_final_state==actionlib::SimpleClientGoalState::ABORTED){
    ROS_INFO("Move_Base_Action_Client::getResult:Aborted 1");
    return 1;
  }else if(move_base_final_state==actionlib::SimpleClientGoalState::PREEMPTED){
    ROS_INFO("Move_Base_Action_Client::getResult:Preempted 2");
    return 2;
  }else{
    ROS_INFO("Move_Base_Action_Client::getResult:Other 3");
    return 3;
  }
}

void Move_Base_Action_Client::initialize(ros::NodeHandle nh){
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Move_Base_Action_Client::initialize: Waiting for the move_base action server to come up");
  }

}

Move_Base_Action_Client::Move_Base_Action_Client(ros::NodeHandle nh):
  ac("/move_base", true)
  {
      clear_costmap.reset(new ros::ServiceClient(nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps")));
    initialize(nh);
}

Move_Base_Action_Client::~Move_Base_Action_Client() {

}
