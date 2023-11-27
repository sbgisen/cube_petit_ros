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
  ArDockingServer ar_docking_server(nh);

  ros::spin();
  return 0;
}


// goalが呼ばれた時に動く
void ArDockingServer::actionServerCallback(const cube_petit_ar_docking::ARDockingGoalConstPtr& goal, Server* as){  
  std::string goal_command = goal->Command.c_str();

  if(goal_command == "charge"){
    //既にドッキングされている場合はワーニングをはいてresultに失敗を返す
    if(ar_docking_controller_.battery_current_monitor.is_charging()){
      ar_docking_controller_.speech_util.sayText("既に充電中です");
      ROS_WARN("IS ALREADY DOCKING");
      actionFinish(FAILED, as);
    }else{
      //ドッキングされていない場合はドッキングをする
      ar_docking_controller_.speech_util.sayText("ドッキングを開始します");
      int docking_result = 0;
      int loop_time = 0;
      while(loop_time < 20){
        loop_time++;
        docking_result = ar_docking_controller_.docking();
        if(!docking_result){
          ar_docking_controller_.speech_util.sayText("ドッキングに成功しました");
          ROS_INFO("loop_time: %d", loop_time);
          ROS_INFO("DOCKING SUCCEED");
          actionFinish(SUCCEED, as);
          break;
        }else{
          ROS_INFO("DOCKING RETRY");
          // ar_docking_controller_.speech_util.sayText("ドッキングに失敗しました");
        }
        if(loop_time == 20){
          ROS_WARN("DOCKING PREMPED");
          actionFinish(PREENMTED, as);
        }       
      }
    } 
  }else if(goal_command == "undock"){
    // //既にドッキングされている場合はワーニングをはいてresultに失敗を返す
    if(!ar_docking_controller_.battery_current_monitor.is_charging()){
      ar_docking_controller_.speech_util.sayText("既にアンドッキングされています");
      ROS_WARN("IS ALREADY UN DOCKING");
      result_.Message = "failed";
      actionFinish(FAILED, as);
    }else{
      ar_docking_controller_.speech_util.sayText("アンドッキングします");
      //ドッキングされていない場合はドッキングをする
      if( !ar_docking_controller_.undocking() ){
        ROS_INFO("UN DOCKING SUCCEED");
        ar_docking_controller_.speech_util.sayText("アンドッキングに成功しました");
        result_.Message = "succeed";
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

void ArDockingServer::actionFinish(int result_int, Server* as){  
  if(result_int == SUCCEED){
    result_.Message = "succeed";
    as->setPreempted(result_, "Docking Succeed");
  }else if( result_int == FAILED){
    result_.Message = "failed";
    as->setAborted(result_, "Docking Failed");
  }else if(result_int == PREENMTED){
    result_.Message = "preempted";
    as->setPreempted(result_, "Docking Preempted");
  }else{
    result_.Message = "error";
    as->setAborted(result_, "Docking Failed");
  }

}

/////////////////////////////
/*  コンストラクタ         */
/////////////////////////////

void ArDockingServer::initialize(ros::NodeHandle nh){
  ROS_INFO("ArDockingServer::initialize");
  // Pregoal_Broadcaster pregoal_broadcaster_(nh);

}

// https://www.gocca.work/cpp-initialize-list/ 参考

ArDockingServer::ArDockingServer(ros::NodeHandle nh):
  server_(nh, "ar_docking_action", boost::bind(&ArDockingServer::actionServerCallback, this, _1, &server_), false),
  pregoal_broadcaster_(nh),
  ar_docking_controller_(nh)
 {
  server_.start();
  ROS_INFO("ar_docking_server_start");
  // std::memset(current_data_, 0.0, sizeof(current_data_));
  initialize(nh);
}

ArDockingServer::~ArDockingServer() {

}
