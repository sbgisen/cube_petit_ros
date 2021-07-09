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
  // PID制御に使用する速度指令値を更新する
  right_target_velocity_ = right_target_velocity;
  left_target_velocity_ = left_target_velocity;
  return 0;
}

// [謎] 電流[A]を引数とし、データにする
// -10[A]~10[A]の電流を-10000~10000で表現するため、1[A]で1000を送る
// double型の引数電流値current_inからint型のdata_outを返す。 
// [TODO]CASTしたら終わり？
int Dji_Can_Communication::current2Data(double current_in){
  int data_out = 0;
  //目標電流値が０以上の場合（正回転の場合？）
  if (current_in >= 0){
    data_out = (int)(current_in*1000);
  // 負回転の場合、符号なし16ビット整数に変換
  }else if (current_in < 0){
    data_out = 0xFFFF+(int)(current_in*1000);
  }
  return data_out;
}

// 左の右のモータのデータからパケットを生成する
int Dji_Can_Communication::createCanPacketAndSend(int left_data,int right_data){
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
    // 左側 ////////////////////////////////////////////////////////////////
    if(msg->id == (0x200 + 0x001) ){
      //減速前の値
      double left_rad = ((msg->data[0] << 8) | msg->data[1]) * 2.0 * M_PI / 8191.0; //0から2*M_PIの範囲

      //減速後の値を出すため、これまでに何回転したかカウントする
      static double left_rad_prev = 0.0;        //前回の位置
      static int left_revolution_count = 0;     //回転数(0〜35。減速比が36のため)

      //正回転
      if( left_rad < 1.0 /4.0 * M_PI && 3.0 / 4.0 * M_PI < left_rad_prev){
        left_revolution_count ++;
        if(left_revolution_count == 36){
          left_revolution_count = 0;
        }
        // ROS_INFO("left_revolution_count++ = %d : %f,%f",left_revolution_count, left_rad, left_rad_prev);
      //負回転
      }else if(left_rad_prev < 1.0 /4.0 * M_PI && 3.0 / 4.0 * M_PI < left_rad){
          left_revolution_count --;
        if(left_revolution_count == -1){
          left_revolution_count = 35;
        }
        // ROS_INFO("left_revolution_count-- = %d : %f,%f",left_revolution_count, left_rad, left_rad_prev);
      }
      left_rad_prev = left_rad;
      
      // 減速後の値(タイヤが何回転したか)を出す
      // 0から(36*2*M_PI)の範囲の値からREDUCTION_RARIOを割り、[0から2*M_PI]までの範囲にする
      double left_rad_reduced = ( left_rad + (2*M_PI*left_revolution_count) ) / REDUCTION_RATIO;
      // Cuboidくん用DiffDriveControllerに合わせるため値の範囲を(-M_PIからM_PI)に変更する
      left_rad_reduced -= M_PI;

      // タイヤのRPMを保存する
      int16_t left_rpm = (msg->data[2] << 8) | msg->data[3];
      double left_rad_per_sec = (double)left_rpm * 2.0 * M_PI / 60.0  / REDUCTION_RATIO;
      // 電流値とトルクを保存する
      double left_current = ( (msg->data[4] << 8) | msg->data[5] ) / 1000.0;
      double left_torque = left_current * TORQUE_COEFFICIENT;
      // 現在の温度を保存する
      double left_temperature = msg->data[6];

      status_[POSITION_LEFT] = left_rad_reduced;
      status_[VELOCITY_LEFT] = left_rad_per_sec ;
      status_[EFFORT_LEFT] = left_torque;
    //右側 //////////////////////////////////////////////////////////////////////////
    }else if(msg->id == (0x200 + 0x002) ){
     //減速前の値
      double right_rad = ((msg->data[0] << 8) | msg->data[1]) * 2.0 * M_PI / 8191.0; //0から2*M_PIの範囲
      //減速後の値を出すため、これまでに何回転したかカウントする
      static double right_rad_prev = 0.0;
      static int right_revolution_count = 0;
      //正回転
      if( right_rad < 1.0 /4.0 * M_PI && 3.0 / 4.0 * M_PI < right_rad_prev){
        right_revolution_count ++;
        if(right_revolution_count == 36){
          right_revolution_count = 0;
        }
        // ROS_INFO("right_revolution_count++ = %d : %f,%f",right_revolution_count, right_rad, right_rad_prev);
      //負回転
      }else if(right_rad_prev < 1.0 /4.0 * M_PI && 3.0 / 4.0 * M_PI < right_rad){
          right_revolution_count --;
        if(right_revolution_count == -1){
          right_revolution_count = 35;
        }
        // ROS_INFO("right_revolution_count-- = %d : %f,%f",right_revolution_count, right_rad, right_rad_prev);
      }
      right_rad_prev = right_rad;
      // 減速後の値(タイヤが何回転したか)を出す
      // 0から(36*2*M_PI)の範囲の値からREDUCTION_RARIOを割り、[0から2*M_PI]までの範囲にする
      double right_rad_reduced = ( right_rad + (2*M_PI*right_revolution_count) ) / REDUCTION_RATIO;
      // 値の範囲を(-M_PIからM_PI)に変更する
      right_rad_reduced -= M_PI;

      // タイヤのRPMを保存する
      int16_t right_rpm = (msg->data[2] << 8) | msg->data[3];
      double right_rad_per_sec = (double)right_rpm * 2.0 * M_PI / 60.0  / REDUCTION_RATIO;
      //電流値とトルクを保存する
      double right_current = ( (msg->data[4] << 8) | msg->data[5] ) / 1000.0;
      double right_torque = right_current * TORQUE_COEFFICIENT;
      //現在の温度を保存する
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

// PID制御
void Dji_Can_Communication::timerCallback(const ros::TimerEvent& event){
  ros::Time time_now = ros::Time::now();

  // 現在の速度と目標速度の更新
  double right_target_velocity = right_target_velocity_;
  double right_velocity = status_[VELOCITY_RIGHT];
  double left_target_velocity = left_target_velocity_;
  double left_velocity = status_[VELOCITY_LEFT];

  // PID制御
  right_target_current_ = -1.0 * right_pid_.updatePid(right_target_velocity - right_velocity, time_now - last_time);
  left_target_current_  = -1.0 *  left_pid_.updatePid(left_target_velocity  -  left_velocity, time_now - last_time);
  //ROS_INFO("timerCallback: right: %f -> %f, left: %f -> %f",right_target_velocity , right_velocity, left_target_velocity  , left_velocity);

  // モータに送る電流値[A]
  double right_target_current = right_target_current_;
  double left_target_current = left_target_current_;
  // 最大電流値を超えないようにする
  if(right_target_current >= MAX_CURRENT){
    right_target_current = MAX_CURRENT;
  }else if(-MAX_CURRENT >= right_target_current){
    right_target_current = -MAX_CURRENT;
  }
  if(left_target_current >= MAX_CURRENT){
    left_target_current = MAX_CURRENT;
  }else if(-MAX_CURRENT >= left_target_current){
    left_target_current = -MAX_CURRENT;
  }

  // 電流指令値をデータに変換する
  double right_data = current2Data(right_target_current);
  double left_data = current2Data(left_target_current);
  // パケットを生成してCANに送信する 
  createCanPacketAndSend(left_data, right_data);
  last_time = time_now;
}


////////////////////////
/* initialize */
////////////////////////
void Dji_Can_Communication::initialize(ros::NodeHandle nh) {
  ROS_INFO("dji_can_communication -> initialize");
  sub_can = nh.subscribe<can_msgs::Frame>("received_messages", 1000, &Dji_Can_Communication::receivedCanCallback, this);
  // ゲインの初期値設定(p, i, d, i_max, i_min)
  left_pid_.initPid(3, 10.0, 0.005, 10.0, -10.0); //i=0
  right_pid_.initPid(3, 10.0, 0.005, 10.0, -10.0);  //i=0
  // 100HzでPID制御のコールバック関数を呼ぶタイマー
  timer = nh.createTimer(ros::Duration(0.01),&Dji_Can_Communication::timerCallback,this);

}

/////////////////////////////
/*  コンストラクタ         */
/////////////////////////////
Dji_Can_Communication::Dji_Can_Communication() {
  TORQUE_COEFFICIENT = 0.18;    //トルク計数
  REDUCTION_RATIO = 36;         //減速比
  MAX_CURRENT = 10.000;         //C610の最大電流
  // MAX_CURRENT = 1.0;            //今だけ
  status_.resize(6,0.0);
  rad_vec.resize(4,0.0);
  right_target_velocity_ = 0;   // 目標速度
  left_target_velocity_ = 0;
  right_target_current_ = 0;    // 目標速度に必要な電流
  left_target_current_ = 0;

  last_time = ros::Time::now();
  ROS_INFO("Dji_Can_Communication::Dji_Can_Communication() -> SUCCEED");
}

Dji_Can_Communication::~Dji_Can_Communication() {
  left_target_current_ = 0.0;
  right_target_current_ = 0.0;
  double right_data = current2Data(right_target_current_);
  double left_data = current2Data(left_target_current_);
  // パケットを生成してCANに送信する 
  createCanPacketAndSend(left_data, right_data);
  sleep(0.05);


}


