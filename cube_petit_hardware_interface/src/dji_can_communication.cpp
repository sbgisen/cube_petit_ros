#include "dji_can_communication.hpp"

////////////////////////////////////////////
//                                        //
//       dji_can_communication.cpp        //
//              2020/11/18                //
//      airi.yokochi@g.softbank.co.jp     //
//                                        //
////////////////////////////////////////////

/*
Motor: DJI M2006 
Motor Controller: C610
*/


//////////////////////////////////////
/*  Send Velocity to CAN */
//////////////////////////////////////
int Dji_Can_Communication::sendVelocityCan(const double left_target_velocity, const double right_target_velocity){
  // 受け取ったらCANに情報を送る　//左はid1、右はid2
  exportCurrentToCan(LEFT+1, velocityToCurrent(left_target_velocity));
  exportCurrentToCan(RIGHT+1, velocityToCurrent(right_target_velocity));  
  return 0;
}

// [謎]
int Dji_Can_Communication::exportCurrentToCan(int id, double send_current){
  //目標電流値が０以上の場合（正回転の場合？）
  if (send_current >= 0){
    Current_16[id-1] = 0x0000+((send_current * current_2_bit_10_)/10000);
    AccessCount[id-1] = 0;
  // 負回転の場合、符号なし8ビット整数に変換
  }else if (send_current < 0){
    Current_16[id-1] = 0xFFFF+((send_current * current_2_bit_10_)/10000);
    AccessCount[id-1] = 0;
  }
  // データ初期化
  for (int i = 0; i < 8; i++){
    if (AccessCount[i] > 100){
      Current_16[i] = 0x0000;
    }
  }

  //
  can_data[0]                 = Current_16[0] >> 8 & 0xFF;
  can_data[1]                 = Current_16[0] & 0xFF;
  can_data[2]                 = Current_16[1] >> 8 & 0xFF;
  can_data[3]                 = Current_16[1] & 0xFF;
  can_data[4]                 = Current_16[2] >> 8 & 0xFF;
  can_data[5]                 = Current_16[2] & 0xFF;
  can_data[6]                 = Current_16[3] >> 8 & 0xFF;
  can_data[7]                 = Current_16[3] & 0xFF;
  if(sendCan(0x200, can_data)) {
        ROS_WARN("write error");
  }
  can_data[0]                 = Current_16[4] >> 8 & 0xFF;
  can_data[1]                 = Current_16[4] & 0xFF;
  can_data[2]                 = Current_16[5] >> 8 & 0xFF;
  can_data[3]                 = Current_16[5] & 0xFF;
  can_data[4]                 = Current_16[6] >> 8 & 0xFF;
  can_data[5]                 = Current_16[6] & 0xFF;
  can_data[6]                 = Current_16[7] >> 8 & 0xFF;
  can_data[7]                 = Current_16[7] & 0xFF;
  if(sendCan(0x1FF,can_data)) {
        ROS_WARN("write error");
  }
  for (int i = 0; i < 8; i++)
  {
    AccessCount[i]++;
  }
  return 0;

}

// 速度指令値から電流値に変換する
int Dji_Can_Communication::velocityToCurrent(double velocity){
  // 角速度　回転速度　電流値を求める
  double radparsec = REDUCTION_RATIO * velocity;
  int rpm = radparsec*60/(2*M_PI);
  int current = 75 * rpm;   //75？

  // 最大電流値を超えないようにする
  if (current > MAX_CURRENT){
    current = MAX_CURRENT;
  }else if (current < -MAX_CURRENT){
    current = -MAX_CURRENT;
  }
  return current;
}

// CANデータをデバイスに送信する。引数はコントローラIDと電流値
int Dji_Can_Communication::sendCan(uint16_t dst_id, uint8_t* data) {
    int                 s; /* can raw socket */
    int                 required_mtu = CAN_MTU;
    int                 enable_canfd = 1;
    struct sockaddr_can addr;
    struct canfd_frame  frame;
    struct ifreq        ifr;

    frame.can_id = dst_id;
    for(int i = 0; i < 8; i++) {
        frame.data[i] = data[i];
    }
    frame.len = 8;

    /* open socket */
    if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("socket");
        return 1;
    }

    // ToDo: Make the interface name (currently constant value can0) configurable
    strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    ifr.ifr_ifindex            = if_nametoindex(ifr.ifr_name);
    if(!ifr.ifr_ifindex) {
        perror("if_nametoindex");
        return 1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    /* disable default receive filter on this RAW socket */
    /* This is obsolete as we do not read from the socket at all, but for */
    /* this reason we can remove the receive list in the Kernel to save a */
    /* little (really a very little!) CPU usage.                          */
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    if(bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        return 1;
    }

    /* send frame */
    if(write(s, &frame, required_mtu) != required_mtu) {
        perror("write");
        ROS_WARN("WRITE ERROR");
        return 1;
    }

    close(s);

    return 0;
}

//////////////////////////////////////
/*  Received Joint State from CAN */
//////////////////////////////////////
void Dji_Can_Communication::updateMotorStatus(std::vector<double>& status_arg){
  //引数のポインタの値を書き換える
  status_arg[VELOCITY_LEFT] = status_[VELOCITY_LEFT];
  status_arg[POSITION_LEFT] = status_[POSITION_LEFT];
  status_arg[VELOCITY_LEFT] = status_[VELOCITY_LEFT];
  status_arg[EFFORT_LEFT] = status_[EFFORT_LEFT];
  status_arg[POSITION_RIGHT] = status_[POSITION_RIGHT];
  status_arg[VELOCITY_RIGHT] = status_[VELOCITY_RIGHT];
  status_arg[EFFORT_RIGHT] = status_[EFFORT_RIGHT];
}

// モータからデータを受け取ったら値を格納するコールバック関数
void Dji_Can_Communication::receivedCanCallback(const can_msgs::Frame::ConstPtr& msg){
  motor_id = 0;
  motor_degree_now = 0;
  motor_rpm = 0;
  motor_current = 0;
  motor_temperature = 0;
  motor_torque = 0;
  motor_rad_per_sec = 0;
  motor_position = 0;

  if(msg->id == (0x200 + 0x001) && msg->dlc == 8 || msg->id == (0x200 + 0x002) && msg->dlc == 8) {  // normal response from motor driver
    motor_id = msg->id;
    motor_degree_now = ((msg->data[0] << 8) | msg->data[1])*360/8191;
    motor_rpm = (msg->data[2] << 8) | msg->data[3];
    motor_current = (msg->data[4] << 8) | msg->data[5];
    motor_temperature = msg->data[6];
    motor_torque = motor_current*TORQUE_COEFFICIENT/1000;
    motor_rad_per_sec = (double)motor_rpm/60*2*M_PI;
    motor_position = 0;

    //ROS_INFO("motor_degree_now [%d], before [%d]", motor_degree_now, motor_degree_before);
 
    // 前回のモータ角度から現在のモータ角度を算出
    if (0<=motor_degree_now && motor_degree_now < 180){    // 角度が0度〜180度 
      if (motor_degree_now <= motor_degree_before && motor_degree_before <= motor_degree_now+180){  // 負回転した
        motor_position -= (motor_degree_before-motor_degree_now);
      }else{  // 正回転したとき1回転していたら360度を足す
        if (motor_degree_now-motor_degree_before < 0){
          motor_position += 360+motor_degree_now-motor_degree_before;
        }else{
          motor_position += motor_degree_now-motor_degree_before;
        }
      }
    }else if (180<= motor_degree_now && motor_degree_now < 360){    // 角度が180度〜360度
      if (motor_degree_now-180 <= motor_degree_before && motor_degree_before <= motor_degree_now){
        motor_position += motor_degree_now-motor_degree_before;
      }else{
        if (motor_degree_before-motor_degree_now < 0){
          motor_position -= (360+motor_degree_before-motor_degree_now);
        }else{
          motor_position -= (motor_degree_before-motor_degree_now);
        }
      }
    }
    motor_degree_before = motor_degree_now;

    if(msg->id == (0x200 + 0x001) ){
      status_[VELOCITY_LEFT] = motor_rad_per_sec;
      status_[POSITION_LEFT] = motor_position;
      status_[EFFORT_LEFT] = motor_torque;
    }else if(msg->id == (0x200 + 0x002) ){
      status_[POSITION_RIGHT] = motor_position;
      status_[VELOCITY_RIGHT] = motor_rad_per_sec;
      status_[EFFORT_RIGHT] = motor_torque;
    }

  }else if(msg->id == (0x200) && msg->dlc == 8 ){
    int current_1 = (msg->data[0] << 8) | msg->data[1];
    int current_2 = (msg->data[2] << 8) | msg->data[3];
    //ROS_INFO("/received_message id: %d ,data1: %d data2: %d", msg->id, current_1, current_2);
  }else if(msg->id == (0x1FF) && msg->dlc == 8){
  
  }else{
    ROS_WARN("Invalid /received_message id: %d", msg->id);
  }
}


/////////////////////////////
/*  コンストラクタ         */
/////////////////////////////
Dji_Can_Communication::Dji_Can_Communication() {
  TORQUE_COEFFICIENT = 0.18;  //トルク計数
  REDUCTION_RATIO = 36;   //減速比
  MAX_CURRENT = 10000;  //C610の最大電流
  status_size_ = 6;
  current_2_bit_10_ = 10000;
  motor_degree_before = 0;

  status_.resize(status_size_,0);
  ROS_INFO("Dji_Can_Communication::Dji_Can_Communication() -> SUCCEED");
}

Dji_Can_Communication::~Dji_Can_Communication() {}

////////////////////////
/* initialize */
////////////////////////
void Dji_Can_Communication::initialize(ros::NodeHandle nh) {
  ROS_INFO("dji_can_communication -> initialize");
  sub_can = nh.subscribe<can_msgs::Frame>("received_messages", 1000, &Dji_Can_Communication::receivedCanCallback, this);
}

