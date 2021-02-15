#include "ar_docking_server.hpp"


////////////////////////////////////////////
//                                        //
//       ar_docking_server.cpp            //
//              2020/12/25                //
//      airi.yokochi@g.softbank.co.jp     //
//                                        //
////////////////////////////////////////////

enum Params{
  SUCCEED,
  FAILED,
  PREENMTED,
  ONGOING,
  FINISHED,
  ABORTED
};



void currentCallback(const std_msgs::Float64::ConstPtr& msg){
  ROS_INFO("%f", msg->data);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "ar_docking_server");
  ros::NodeHandle nh("~");
  AR_Docking_Server ar_docking_server(nh);

  ros::spin();
  return 0;
}


// goalが呼ばれた時に動く
void AR_Docking_Server::actionServerCallback(const cube_petit_ar_docking::ARDockingGoalConstPtr& goal, Server* as){  
  std::string goal_command = goal->Command.c_str();

  if(goal_command == "charge"){
    //既にドッキングされている場合はワーニングをはいてresultに失敗を返す
    if(ar_docking_controller.battery_current_monitor.is_charging()){
      ROS_WARN("IS ALREADY DOCKING");
      actionFinish(FAILED, as);
    }else{
      //ドッキングされていない場合はドッキングをする
      int docking_result = 0;
      int loop_time = 0;
      while(loop_time < 5){
        loop_time++;
        docking_result = ar_docking_controller.docking();
        if(!docking_result){
          ROS_INFO("DOCKING SUCCEED");
          actionFinish(SUCCEED, as);
          break;
        }else{
          ROS_INFO("DOCKING RETRY");
        }
        if(loop_time == 4){
          ROS_WARN("DOCKING PREMPED");
          actionFinish(PREENMTED, as);
        }       
      }
    } 
  }else if(goal_command == "undock"){
    // //既にドッキングされている場合はワーニングをはいてresultに失敗を返す
    if(!ar_docking_controller.battery_current_monitor.is_charging()){
      ROS_WARN("IS ALREADY UN DOCKING");
      result.Message = "failed";
      actionFinish(FAILED, as);
    }else{
      //ドッキングされていない場合はドッキングをする
      if( !ar_docking_controller.undocking() ){
        ROS_INFO("UN DOCKING SUCCEED");
        result.Message = "succeed";
        actionFinish(SUCCEED, as);
      }else{
         ROS_WARN("UN DOCKING PREMPED");
         actionFinish(PREENMTED, as);
      }
    }
  }else{
    ROS_WARN("goal->Command is Invalid");
    as->setAborted();
  }

}

void AR_Docking_Server::actionFinish(int result_int, Server* as){  
  if(result_int == SUCCEED){
    result.Message = "succeed";
    as->setPreempted(result, "Docking Succeed");
  }else if( result_int == FAILED){
    result.Message = "failed";
    as->setAborted(result, "Docking Failed");
  }else if(result_int == PREENMTED){
    result.Message = "preempted";
    as->setPreempted(result, "Docking Preempted");
  }else{
    result.Message = "error";
    as->setAborted(result, "Docking Failed");
  }

}

/////////////////////////////
/*  コンストラクタ         */
/////////////////////////////

void AR_Docking_Server::initialize(ros::NodeHandle nh){
  ROS_INFO("AR_Docking_Server::initialize");
  // Pregoal_Broadcaster pregoal_broadcaster(nh);

}

// https://www.gocca.work/cpp-initialize-list/ 参考

AR_Docking_Server::AR_Docking_Server(ros::NodeHandle nh):
  server(nh, "ar_docking_action", boost::bind(&AR_Docking_Server::actionServerCallback, this, _1, &server), false),
  pregoal_broadcaster(nh),
  ar_docking_controller(nh)
 {
  server.start();
  ROS_INFO("ar_docking_server_start");
  // std::memset(current_data, 0.0, sizeof(current_data));
  initialize(nh);
}

AR_Docking_Server::~AR_Docking_Server() {

}


