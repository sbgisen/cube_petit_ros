#include "cube_petit_hardware_interface.hpp"

////////////////////////////////////////////
//                                        //
//      cube_petit_hardware_interface     //
//              2020/11/18                //
//      airi.yokochi@g.softbank.co.jp     //
//                                        //
////////////////////////////////////////////

/*
Motor: DJI M2006 
Motor Controller: C610
*/

int main(int argc, char **argv){
  ros::init(argc, argv, "cube_petit_hardware_interface");
  ros::NodeHandle nh("~");

  // Cube_Petit_Hardware_Interfaceクラスの初期化
  Cube_Petit_Hardware_Interface hw_interface;
  hw_interface.initialize(nh);

  // Controller_Managerクラスの初期化
  controller_manager::ControllerManager controller_manager(&hw_interface, nh);

  // loop_rateは　1/0.01 = 100Hz
  ros::Rate loop_rate(1.0 / hw_interface.getPeriod().toSec());

  // http://docs.ros.org/en/api/roscpp/html/classros_1_1AsyncSpinner.html
  ros::AsyncSpinner spinner(1);           // 別スレッドを1つ立ててspinする　[TODO]排他処理必要
  spinner.start();
  while(ros::ok()){
    // Update all active controllers. NEED REAL-TIME SAFE FUCTION
    // http://docs.ros.org/en/jade/api/controller_manager/html/c++/classcontroller__manager_1_1ControllerManager.html
    controller_manager.update(hw_interface.getTime(), hw_interface.getPeriod());
    ros::spinOnce();                      // AsyncSpinnerを使わない場合は必要
    hw_interface.update();                // ReadとWriteを呼び出す
    loop_rate.sleep();
  }
  spinner.stop();
  return 0;
}

// シェル実行
std::string Cube_Petit_Hardware_Interface::execCmd(std::string system_cmd){ //static
  FILE *fp;
  //ROS_INFO("[%s]" system_cmd);
  std::string result = "";
  char buf[256];
  if( (fp = popen(system_cmd.c_str(),"r")) != NULL )
  {
    fgets(buf, 256, fp);
    pclose(fp);
    result = buf;
  }
  return result;
}

////////////////////////////////////////////
/*  read Joint States from CAN          */
////////////////////////////////////////////
void Cube_Petit_Hardware_Interface::read(){
  //std::lock_guard<std::mutex> lock(mutex);  //ロックする
  // ROS_INFO("Cube_Petit_Hardware_Interface::read -> START");
  std::vector<double> status;
  status.resize(dji_can_.status_size_, 0);

  // 
  dji_can_.updateMotorStatus(status);

  // copies and updates the status
  position_.at(LEFT)  = status.at(POSITION_LEFT);
  velocity_.at(LEFT)  = status.at(VELOCITY_LEFT);
  effort_.at(LEFT)    = status.at(EFFORT_LEFT);
  position_.at(RIGHT) = status.at(POSITION_RIGHT);
  velocity_.at(RIGHT) = status.at(VELOCITY_RIGHT);
  effort_.at(RIGHT)   = status.at(EFFORT_RIGHT);

  //モータの向きを逆にする
  //atは引数番目の要素を取得する
  if(direction_.at(LEFT) == false){
     position_.at(LEFT) *= -1.0;
     velocity_.at(LEFT) *= -1.0;
     effort_.at(LEFT) *= - 1.0;
  }
  if(direction_.at(RIGHT) == false){
    position_.at(RIGHT) *= -1.0;
    effort_.at(RIGHT) *= - 1.0;
    velocity_.at(RIGHT) *= -1.0;
  }
  // ROS_INFO("Cube_Petit_Hardware_Interface::read   -> position %f, %f", position_.at(LEFT), position_.at(RIGHT));
  // ROS_INFO("Cube_Petit_Hardware_Interface::read   -> velocity %f, %f", velocity_.at(LEFT), velocity_.at(RIGHT));
  // ROS_INFO("Cube_Petit_Hardware_Interface::read   -> effort %f, %f", effort_.at(LEFT), effort_.at(RIGHT));
  // ROS_INFO("Cube_Petit_Hardware_Interface::read -> FINISH");

}

////////////////////////////////////////////////////
/*  write target Velocity Commamd to CAN          */
////////////////////////////////////////////////////
void Cube_Petit_Hardware_Interface::write(){
  //std::lock_guard<std::mutex> lock(mutex);  //ロックする
  // ROS_INFO("Cube_Petit_Hardware_Interface::write -> START");

  // sets commands depending on its direction
  if(direction_.at(LEFT)) {
    if(direction_.at(RIGHT)) {
      dji_can_.sendVelocityCan( velocity_command_.at(LEFT),  velocity_command_.at(RIGHT));
    } else {
      dji_can_.sendVelocityCan( velocity_command_.at(LEFT), -velocity_command_.at(RIGHT));
    }
  } else {
    if(direction_.at(RIGHT)) {
      dji_can_.sendVelocityCan(-velocity_command_.at(LEFT),  velocity_command_.at(RIGHT));
    } else {
      dji_can_.sendVelocityCan(-velocity_command_.at(LEFT), -velocity_command_.at(RIGHT));
    }
  }
  // ROS_INFO("Cube_Petit_Hardware_Interface::write -> FINISH");
}

/////////////////////////
/*  main loop          */
/////////////////////////
void Cube_Petit_Hardware_Interface::update(){
  //std::lock_guard<std::mutex> lock(mutex);  //ロックする
  read();
  write();
}

/////////////////////////////////
/*  コンストラクタ・初期化     */
/////////////////////////////////
Cube_Petit_Hardware_Interface::Cube_Petit_Hardware_Interface(){
  // モータ位置。モータ角速度、モータトルクを0.0で初期化する
  position_.resize(2, 0.0);
  velocity_.resize(2, 0.0);
  effort_.  resize(2, 0.0);

  // 速度指令値を0.0で初期化する
  velocity_command_.resize(2, 0.0);

  // 右のモータをfalseとして速度指令値の正負を反転する
  direction_.resize(2);
  direction_.at(LEFT)  = true;
  direction_.at(RIGHT) = false;

  // state_handleは引数()
  hardware_interface::JointStateHandle state_handle_left ("left_wheel_joint",  &position_[LEFT],  &velocity_[LEFT],  &effort_[LEFT]);
  hardware_interface::JointStateHandle state_handle_right("right_wheel_joint", &position_[RIGHT], &velocity_[RIGHT], &effort_[RIGHT]);

  // あ
  joint_state_interface_.registerHandle(state_handle_left);
  joint_state_interface_.registerHandle(state_handle_right);
  registerInterface(&joint_state_interface_);

  hardware_interface::JointHandle velocity_handle_left (joint_state_interface_.getHandle("left_wheel_joint"), &velocity_command_[LEFT]);
  hardware_interface::JointHandle velocity_handle_right(joint_state_interface_.getHandle("right_wheel_joint"), &velocity_command_[RIGHT]);

  // 
  joint_velocity_interface_.registerHandle(velocity_handle_left);
  joint_velocity_interface_.registerHandle(velocity_handle_right);

  registerInterface(&joint_velocity_interface_);
}

// Dji_Can_Communicationクラスのinitializeを呼び出す関数
void Cube_Petit_Hardware_Interface::initialize(ros::NodeHandle nh){
  dji_can_.initialize(nh);
}

Cube_Petit_Hardware_Interface::~Cube_Petit_Hardware_Interface(){
}

// ROS上の現在時間をros::Time型で返す
ros::Time Cube_Petit_Hardware_Interface::getTime() {
    return ros::Time::now();
}

// 0.01秒間をros::Duration型で返す
ros::Duration Cube_Petit_Hardware_Interface::getPeriod() {
    return ros::Duration(0.01);
}