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
  // 受け取ったら左右の速度指令値をデータに変換する　//左はid1、右はid2
  int right_data = velocity2Data(right_target_velocity);
  int left_data  = velocity2Data(left_target_velocity);
  // パケットを生成してCANに送信する 
  createCanPacketAndSend(right_data,left_data);  
  return 0;
}
// [謎] 電流値をデータにする
int Dji_Can_Communication::current2Data(double current_in){
  // データ初期化
  // double型の引数電流値current_inからint型のdata_outを返す。
  int data_out = 0;
  //目標電流値が０以上の場合（正回転の場合？）
  if (current_in >= 0){
    data_out = (int)(current_in);
  // 負回転の場合、符号なし8ビット整数に変換
  }else if (current_in < 0){
    data_out = 0xFFFF+(int)(current_in);
  }
  return data_out;
}

// [謎] 速度指令値をデータにする
int Dji_Can_Communication::velocity2Data(double velocity_in){
  // データ初期化
  int data_out = 0;
  // 角速度、回転速度から速度値を求める
  double radparsec = REDUCTION_RATIO * velocity_in;
  double rpm = radparsec * 60.0 / ( 2.0 * M_PI);
  // https://rm-static.djicdn.com/tem/RM%20C610%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E8%B0%83%E9%80%9F%E5%99%A8%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%20%E5%8F%91%E5%B8%83%E7%89%88.pdf
  // p.9 P:出力の角度　[k*rpm*T]
  int velocity_data = (int)( rpm * 75.0 ); // [調べる] what is 75mm ????
  //目標電流値が０以上の場合（正回転の場合？）
  if (velocity_data >= 0){
    data_out = (int)(velocity_data);
  // 負回転の場合、符号なし8ビット整数に変換
  }else if (velocity_data < 0){
    data_out = 0xFFFF+(int)(velocity_data);
  }
  return data_out;
}

// 左の右のモータのデータからパケットを生成する。[謎]
int Dji_Can_Communication::createCanPacketAndSend(int right_data,int left_data){
  can_data[0]                 = left_data >> 8 & 0xFF;
  can_data[1]                 = left_data & 0xFF;
  can_data[2]                 = right_data >> 8 & 0xFF;
  can_data[3]                 = right_data & 0xFF;
  can_data[4]                 = 0;
  can_data[5]                 = 0;
  can_data[6]                 = 0;
  can_data[7]                 = 0;
  if(sendCan(0x200, can_data)) {
        ROS_WARN("write error");
  }
  return 0;
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
  if(msg->dlc == 8){
    // 左側
    if(msg->id == (0x200 + 0x001) ){
      //減速前の値
      double left_rad = ((msg->data[0] << 8) | msg->data[1]) * 2.0 * M_PI / 8191.0; //0から2*M_PIの範囲
      // // -M_PIからM_PIの範囲に変換する
      // if(left_rad >= M_PI){
      //   //left_rad -= (2.0 * M_PI) * (int)((left_rad + M_PI) / (2.0 * M_PI));
      //   left_rad -= (2.0 * M_PI);
      // }else if(left_rad < -M_PI){
      //   //left_rad += (2.0 * M_PI) * (int)(-1.0 * (left_rad - M_PI) / (2.0 * M_PI));
      //   left_rad +=  2.0 * M_PI;
      // }

      //減速後の値を出すため、これまでに何回転したかカウントする
      static double left_rad_prev = 0.0;
      static int left_revolution_count = 0;
      double left_rad_total;

      if( left_rad < 1.0 /4.0 * M_PI && 3.0 / 4.0 * M_PI < left_rad_prev){// && left_rad_prev - left_rad <= M_PI ){//TODO*BUGFIX
        left_revolution_count ++;
        if(left_revolution_count == 36){
          left_revolution_count = 0;
        }
        ROS_INFO("left_revolution_count++ = %d : %f,%f",left_revolution_count, left_rad, left_rad_prev);
      }else if(left_rad_prev < 1.0 /4.0 * M_PI && 3.0 / 4.0 * M_PI < left_rad){ // && left_rad - left_rad_prev <= M_PI){
          left_revolution_count --;
        if(left_revolution_count == -1){
          left_revolution_count = 35;
        }
        ROS_INFO("left_revolution_count-- = %d : %f,%f",left_revolution_count, left_rad, left_rad_prev);
      }

      // if( (left_rad_prev - left_rad) >= M_PI ){//TODO*BUGFIX
      //   left_revolution_count ++;
      //   ROS_INFO("left_revolution_count = %d,%f,%f",left_revolution_count, left_rad, left_rad_prev);
      // }else if((left_rad_prev - left_rad) < -1.0*M_PI){
      //   left_revolution_count --;
      //   ROS_INFO("left_revolution_count = %d,%f,%f",left_revolution_count, left_rad, left_rad_prev);

      // }
      left_rad_prev = left_rad;
      //減速後の値(タイヤが何回転したか)を出す
      //0から36*2*M_PIの値/36で0から2*M_PIまでの値になる
      double left_rad_reduced = ( left_rad + (2*M_PI*left_revolution_count) ) / REDUCTION_RATIO;
      left_rad_reduced -= M_PI;
      // static int print_count = 0;
      // print_count++;
      // if(print_count % 10 == 0){
      //   ROS_INFO("left_rad_reduced = %f",left_rad_reduced);
      // }

      int16_t left_rpm = (msg->data[2] << 8) | msg->data[3];
      double left_rad_per_sec = (double)left_rpm * 2.0 * M_PI / 60.0  / REDUCTION_RATIO;

      double left_current = ( (msg->data[4] << 8) | msg->data[5] ) / 1000.0;
      double left_torque = left_current * TORQUE_COEFFICIENT;

      double left_temperature = msg->data[6];

      status_[POSITION_LEFT] = left_rad_reduced;
      status_[VELOCITY_LEFT] = left_rad_per_sec ;
      status_[EFFORT_LEFT] = left_torque;
    //右側
    }else if(msg->id == (0x200 + 0x002) ){
     //減速前の値
      double right_rad = ((msg->data[0] << 8) | msg->data[1]) * 2.0 * M_PI / 8191.0; //0から2*M_PIの範囲
      //減速後の値を出すため、これまでに何回転したかカウントする
      static double right_rad_prev = 0.0;
      static int right_revolution_count = 0;
      double right_rad_total;

      if( right_rad < 1.0 /4.0 * M_PI && 3.0 / 4.0 * M_PI < right_rad_prev){
        right_revolution_count ++;
        if(right_revolution_count == 36){
          right_revolution_count = 0;
        }
        ROS_INFO("right_revolution_count++ = %d : %f,%f",right_revolution_count, right_rad, right_rad_prev);
      }else if(right_rad_prev < 1.0 /4.0 * M_PI && 3.0 / 4.0 * M_PI < right_rad){
          right_revolution_count --;
        if(right_revolution_count == -1){
          right_revolution_count = 35;
        }
        ROS_INFO("right_revolution_count-- = %d : %f,%f",right_revolution_count, right_rad, right_rad_prev);
      }
      right_rad_prev = right_rad;
      //減速後の値(タイヤが何回転したか)を出す
      //0から36*2*M_PIの値/36で0から2*M_PIまでの値になる
      double right_rad_reduced = ( right_rad + (2*M_PI*right_revolution_count) ) / REDUCTION_RATIO;
      right_rad_reduced -= M_PI;



      int16_t right_rpm = (msg->data[2] << 8) | msg->data[3];
      double right_rad_per_sec = (double)right_rpm * 2.0 * M_PI / 60.0  / REDUCTION_RATIO;
      double right_current = ( (msg->data[4] << 8) | msg->data[5] ) / 1000.0;
      double right_torque = right_current * TORQUE_COEFFICIENT;
      double right_temperature = msg->data[6];

      status_[POSITION_RIGHT] = right_rad_reduced;
      status_[VELOCITY_RIGHT] = right_rad_per_sec;
      status_[EFFORT_RIGHT] = right_torque;
     }else if(msg->id == (0x200)){
      int current_1 = (msg->data[0] << 8) | msg->data[1];
      int current_2 = (msg->data[2] << 8) | msg->data[3];
      //ROS_INFO("/received_message id: %d ,data1: %d data2: %d", msg->id, current_1, current_2);
    }else if(msg->id == (0x1FF)){
      //ROS_INFO("/received_message id: %d", msg->id);
    }else if(msg->id == (0x004)){
      ROS_WARN("Controller could not connect to motors.");     
    }else{
      ROS_WARN("Invalid /received_message id: %d", msg->id);
    }
  }else{
      ROS_WARN("Invalid /received_message dlc: %d", msg->dlc);
  }
}


/////////////////////////////
/*  コンストラクタ         */
/////////////////////////////
Dji_Can_Communication::Dji_Can_Communication() {
  TORQUE_COEFFICIENT = 0.18;  //トルク計数
  REDUCTION_RATIO = 36;   //減速比
  MAX_CURRENT = 10.000;  //C610の最大電流
  MAX_CURRENT = 3.0; //今だけ
  // motor_degree_before = 0;
  // motor_rad_before = 0;
  // motor_rad_before_left = 0;
  // motor_rad_before_right = 0;
  status_.resize(6,0.0);
  rad_vec.resize(4,0.0);
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

