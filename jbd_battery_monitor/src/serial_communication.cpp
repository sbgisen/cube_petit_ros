#include "serial_communication.hpp"


////////////////////////////////////////////
//                                        //
//         serial_communication            //
//              2020/12/25                //
//      airi.yokochi@g.softbank.co.jp     //
//                                        //
////////////////////////////////////////////

int Serial_Communication::serial_parse(std::vector<uint8_t>& data){
  std::size_t data_size = data.size();

  // ROS_INFO("Serial_Communication::serial_parse:Start");
  // for(auto a: data){
  //   printf("%02x ",a);
  // }
  // std::cout << std::endl;

  if(data[2] == 0x80){
    ROS_WARN("Serial_Communication::serial_parse::return error 0x80");
  }else if(data[2] != 0x00){
    ROS_WARN("Serial_Communication::serial_parse::invalid data");
  }

  // Get Whole Volume, Current, Remaining
  if(data[1] == 0x03){
    ROS_INFO("Serial_Communication::serial_parse::Battery Data");
    uint16_t battery_voltage_16   = data[4] << 8 | data[5];
    uint16_t charge_current_16    = data[6] << 8 | data[7];
    uint16_t battery_remaining_16 = data[8] << 8 | data[9];

    // [10mV]から[V]に変換する
    battery_voltage = ((int)battery_voltage_16*10)/1000.0;
    charge_current = ((int)charge_current_16*10)/1000.0;
    battery_remaining = ((int)battery_remaining_16*10)/1000.0;

    ROS_INFO("Serial_Communication::Voltage: %f", battery_voltage);
    ROS_INFO("Serial_Communication::Current: %f", charge_current);
    ROS_INFO("Serial_Communication::Remain: %f", battery_remaining);

  // Get Each Cell Volume
  }else if(data[1] == 0x04){
    ROS_INFO("Serial_Communication::serial_parse::Each Volume");
    uint8_t cell_num = data[3];
    // Check Number of Cells もしセル数が4ではなかったらワーニング
    if(cell_num == 0x08){
      uint16_t cell_volume_16_1 = data[4] << 8 | data[5];
      uint16_t cell_volume_16_2 = data[6] << 8 | data[7];
      uint16_t cell_volume_16_3 = data[8] << 8 | data[9];
      uint16_t cell_volume_16_4 = data[10] << 8 | data[11];

      double cell_volume_1 = ((int)cell_volume_16_1)/1000.0;
      double cell_volume_2 = ((int)cell_volume_16_2)/1000.0;
      double cell_volume_3 = ((int)cell_volume_16_3)/1000.0;
      double cell_volume_4 = ((int)cell_volume_16_4)/1000.0;

      ROS_INFO("Serial_Communication::Voltage 1: %f", cell_volume_1);
      ROS_INFO("Serial_Communication::Voltage 2: %f", cell_volume_2);
      ROS_INFO("Serial_Communication::Voltage 3: %f", cell_volume_3);
      ROS_INFO("Serial_Communication::Voltage 4: %f", cell_volume_4);
    }else{
      ROS_WARN("Serial_Communication::serial_parse::Vattery Cell is only %x", cell_num);
      return 1;
    }

  }
  return 0;
}

int Serial_Communication::serial_send(uint8_t* data){
  // ROS_INFO("Serial_Communication::serial_send");
  write(fd, (char*)data, sizeof(data)/sizeof(*data));
  return 0;
}
 

//シリアル受信：データを読み込んで0xDDから始まり0x77で終わればserial_parseを呼ぶ
int Serial_Communication::serial_read(){
  int req_size = 1;                                   //1バイト読み込む
  int available_size = 0;                             //バッファに溜まっているデータ数を取得する
  ioctl(fd, FIONREAD, &available_size);

  if(available_size == 0){
    return 1;
  }

  // ROS_INFO("Serial_Communication::avaiable_size: %d", available_size);

  //[TODO ]バッファにデータが溜まりすぎていたら消す

  //受信待機が発生しないようにする（いらんかも)
  int read_size;
  if(req_size > available_size) {
      read_size = read(fd, &buf, available_size);
      // ROS_INFO("Serial_Communication::[%02x]", buf);
  } else {
      read_size = read(fd, &buf, req_size);
      // ROS_INFO("Serial_Communication::[%02x]", buf);
  }

  //0xDDがデータ開始バイト
  if(buf == 0xdd){
    data_start_flag = true;
    // ROS_INFO("Serial_Communication::data_start");
  }
  //0X77がデータの終了バイト
  if(data_start_flag && buf == 0x77){
    data_end_flag = true;
    // ROS_INFO("Serial_Communication::data_end");
  }

  //最後は格納してからデータをクリアする
  if(data_end_flag){
    data_vector.push_back(buf);
    serial_parse(data_vector);
    data_vector.clear();
    data_start_flag = false;
    data_end_flag = false;
  }
  //0xDDから始まっていればデータに格納する
  if(data_start_flag){
    data_vector.push_back(buf);
  }

  return 0;

}


// 
void Serial_Communication::updateBatteryStatus(std::vector<double>& battery_status){
  requset_loop();
  battery_status[BATTERY_REMAINING] = battery_remaining;
  battery_status[BATTERY_VOLTAGE] = battery_voltage;
  battery_status[CHARGE_CURRENT] = charge_current;
}

// リクエストを書く
void Serial_Communication::requset_loop(){
  requsetBatteryStatus();
  // requestEachBatteryVoltage();
}

// バッテリの総電圧、電流、残り容量をリクエストするパケットの送信
int Serial_Communication::requsetBatteryStatus(){
  uint8_t request_battery_status[7];
  request_battery_status[0] = 0xDD;
  request_battery_status[1] = 0xA5;
  request_battery_status[2] = 0x03;
  request_battery_status[3] = 0x00;
  request_battery_status[4] = 0xFF;
  request_battery_status[5] = 0xFD;
  request_battery_status[6] = 0x77;

  if( serial_send(request_battery_status) ){
    ROS_WARN("Serial_Communication::requestBatteryStatus::cannnot send data");
  }
  return 0;
}

// 各セルの電圧をリクエストするパケットの送信
int Serial_Communication::requestEachBatteryVoltage(){
  uint8_t request_battery_voltage[7];
  request_battery_voltage[0] = 0xDD;
  request_battery_voltage[1] = 0xA5;
  request_battery_voltage[2] = 0x04;
  request_battery_voltage[3] = 0x00;
  request_battery_voltage[4] = 0xFF;
  request_battery_voltage[5] = 0xFC;
  request_battery_voltage[6] = 0x77;

  if(serial_send(request_battery_voltage) ){
    ROS_WARN("Serial_Communication::requestEachBatteryVoltage::cannnot send data");
  }

  return 0;
}


// シリアル通信の開始
int Serial_Communication::serial_initialize()
{
  struct termios tio;
  memset(&tio,0,sizeof(tio));
  tio.c_cflag = CS8 | CLOCAL | CREAD;
  tio.c_cc[VTIME] = 100;



  int serial_open_count;
  for(serial_open_count=0; serial_open_count< 5 ; serial_open_count++){
    if(serial_open() < 0){
      ROS_WARN("Serial_Communication::Could not open port: /dev/ttyUSB0");
    }else{
      ROS_INFO("Serial_Communication::Open port: /dev/ttyUSB0");
      is_usb_connected = 1;

      tio.c_cflag += CREAD;               // 受信有効
      tio.c_cflag += CLOCAL;              // ローカルライン（モデム制御なし）
      tio.c_cflag += CS8;                 // データビット:8bit
      tio.c_cflag += 0;                   // ストップビット:1bit
      tio.c_cflag += 0;                   // パリティ:None
      cfsetispeed( &tio, BAUD_RATE );
      cfsetospeed( &tio, BAUD_RATE );
      cfmakeraw(&tio);                    // RAWモード
      tcsetattr( fd, TCSANOW, &tio );     // デバイスに設定を行う
      ioctl(fd, TCSETS, &tio);            // ポートの設定を有効にする
      //serial_read();
      return 0;
    }
  }
  ROS_WARN("Serial_Communication::serial_initialize: failed");
  is_usb_connected = 0;
  return 1;

}

// ポートを開く
int Serial_Communication::serial_open(){
  fd = open(SERIAL_PORT,O_RDWR);
  return fd;
}

// ポートを閉じる
int Serial_Communication::serial_close(){
  close(fd);
}

// 疎通確認
int Serial_Communication::serial_ping(){
  return 0;
}

// [TODO] fdの再初期化
int Serial_Communication::serial_reconnect(){
  // close(fd);
  // serial_initialize();
  return 0;
}


Serial_Communication::Serial_Communication(){
  battery_remaining = 0.0;
  battery_voltage = 0.0;
  charge_current = 0.0;
  data_count = 0;
  data_start_flag = false;
  data_end_flag = false;
}


Serial_Communication::~Serial_Communication(){    
  serial_close();
}
