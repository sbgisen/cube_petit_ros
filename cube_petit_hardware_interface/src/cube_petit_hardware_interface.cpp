#include "cube_petit_hardware_interface.hpp"

int main(int argc, char **argv){
  ros::init(argc, argv, "cube_petit_hardware_interface");
  ros::NodeHandle nh("~");
  ros::Rate loop_rate(1);
  int count =1;

  Cube_Petit_Hardware_Interface hw_interface(nh);
  // 実機あり


  // [TODO]インスタンス化する前にCANのせっとを完了させるMUTEX処理
  // CAN通信のセットをする
  std::string cmd_result = "-1";
  cmd_result = hw_interface.execCmd("echo gisen55 | sudo -S bash ~/ros/src/sbgisen/raspberrypi_cube_moc/cube_petit_hardware_interface/setCan0.sh");
  //結果は 0(succss), 1(find can0 failed)  or 3(set can0 failed)
  // 2回目以降は失敗する
  cmd_result = "0";

  if(cmd_result.c_str() != "0"){
    ROS_WARN("Failed to execCmd result :%s", cmd_result.c_str());
    //return 0;
  }else{
    ROS_INFO("Success to execCmd result :%s", cmd_result.c_str());
  }

  while(ros::ok()){
    ROS_INFO("hello");
    hw_interface.update();
    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }



    return 0;
}

std::string Cube_Petit_Hardware_Interface::execCmd(std::string system_cmd){ //static
  FILE *fp;
//  ROS_INFO("[%s]" system_cmd);
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

void Cube_Petit_Hardware_Interface::read(){
  std::vector<double> status;
  status.resize(dji_can_.status_size_, 0);

  // [TODO] エラー吐く
  dji_can_.updateMotorStatus(status);

  //モータの向きを逆にする
  //atは引数番目の要素を取得する
    if(direction_.at(LEFT) == false){
        status.at(POSITION_LEFT) *= -1.0;
        status.at(VELOCITY_LEFT) *= -1.0;
        status.at(EFFORT_LEFT)   *= -1.0;
    }

    if(direction_.at(RIGHT) == false){
        status.at(POSITION_RIGHT) *= -1.0;
        status.at(VELOCITY_RIGHT) *= -1.0;
        status.at(EFFORT_RIGHT)   *= -1.0;
    }
    // copies and updates the status
    position_.at(LEFT)  = status.at(POSITION_LEFT);
    position_.at(RIGHT) = status.at(POSITION_RIGHT);
    velocity_.at(LEFT)  = status.at(VELOCITY_LEFT);
    velocity_.at(RIGHT) = status.at(VELOCITY_RIGHT);
    effort_.at(LEFT)    = status.at(EFFORT_LEFT);
    effort_.at(RIGHT)   = status.at(EFFORT_RIGHT);

}

void Cube_Petit_Hardware_Interface::write(){
  // PCからモータへ書き込む`Joint State`の値をチェックして値が変わっていたら,dji_can_.setCanを呼び出す
  
  // "Motor_1/joint_state_msg"トピックを受け取ったら呼び出される関数 */
  
  //Motor1がright Motor2がleft
  //右のタイヤのvelocityの正負を反転させる
  
  //[LEFT,velocity]
  //dji_can_.Send_Velocity_Can();
      // sets commands depending on its direction
    if(direction_.at(LEFT)) {
        if(direction_.at(RIGHT)) {
            dji_can_.send_Velocity_Can( velocity_command_.at(LEFT),  velocity_command_.at(RIGHT));
        } else {
            dji_can_.send_Velocity_Can( velocity_command_.at(LEFT), -velocity_command_.at(RIGHT));
        }
    } else {
        if(direction_.at(RIGHT)) {
            dji_can_.send_Velocity_Can(-velocity_command_.at(LEFT),  velocity_command_.at(RIGHT));
        } else {
            dji_can_.send_Velocity_Can(-velocity_command_.at(LEFT), -velocity_command_.at(RIGHT));
        }
    }

}

void Cube_Petit_Hardware_Interface::update(){
    
    dji_can_.loop_ros();
    //read();
    //write();
}


Cube_Petit_Hardware_Interface::Cube_Petit_Hardware_Interface(ros::NodeHandle nh){
  //dji_can_.reset(new Dji_Can_Communication(nh));
  dji_can_.initialize(nh);

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
  hardware_interface::JointStateHandle state_handle_left ("wheel_left_joint",  &position_[LEFT],  &velocity_[LEFT],  &effort_[LEFT]);
  hardware_interface::JointStateHandle state_handle_right("wheel_right_joint", &position_[RIGHT], &velocity_[RIGHT], &effort_[RIGHT]);

  // あ
  joint_state_interface_.registerHandle(state_handle_left);
  joint_state_interface_.registerHandle(state_handle_right);
  registerInterface(&joint_state_interface_);

  hardware_interface::JointHandle velocity_handle_left (joint_state_interface_.getHandle("wheel_left_joint"),  &velocity_command_[LEFT]);
  hardware_interface::JointHandle velocity_handle_right(joint_state_interface_.getHandle("wheel_right_joint"), &velocity_command_[RIGHT]);

  // 
  joint_velocity_interface_.registerHandle(velocity_handle_left);
  joint_velocity_interface_.registerHandle(velocity_handle_right);

  registerInterface(&joint_velocity_interface_);
}

Cube_Petit_Hardware_Interface::~Cube_Petit_Hardware_Interface(){
}

