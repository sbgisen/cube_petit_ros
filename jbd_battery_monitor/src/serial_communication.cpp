#include "serial_communication.hpp"

////////////////////////////////////////////
//                                        //
//         serial_communication            //
//              2020/12/25                //
//      airi.yokochi@g.softbank.co.jp     //
//                                        //
////////////////////////////////////////////

int SerialCommunication::serialParse(std::vector<uint8_t>& data)
{
  std::size_t data_size = data.size();

  // ROS_INFO("SerialCommunication::serialParse:Start");
  // for(auto a: data){
  //   printf("%02x ",a);
  // }
  // std::cout << std::endl;

  if (data[2] == 0x80)
  {
    ROS_WARN("SerialCommunication::serialParse::return error 0x80");
  }
  else if (data[2] != 0x00)
  {
    ROS_WARN("SerialCommunication::serialParse::invalid data");
  }

  // Get Whole Volume, Current, Remaining
  if (data[1] == 0x03)
  {
    ROS_DEBUG("SerialCommunication::serialParse::Battery Data");
    uint16_t battery_voltage_16 = data[4] << 8 | data[5];
    int16_t charge_current_16 = data[6] << 8 | data[7];
    uint16_t battery_remaining_16 = data[8] << 8 | data[9];

    // [10mV]から[V]に変換する
    battery_voltage_ = ((int)battery_voltage_16 * 10) / 1000.0;
    charge_current_ = ((int)charge_current_16 * 10) / 1000.0;
    battery_remaining_ = ((int)battery_remaining_16 * 10) / 1000.0;
    ROS_INFO("--------------------------------------------------");
    ROS_INFO("SerialCommunication::Voltage: %f", battery_voltage_);
    ROS_INFO("SerialCommunication::Current: %f", charge_current_);
    ROS_INFO("SerialCommunication::Remain: %f", battery_remaining_);

    uint16_t kousyou_remaining = data[10] << 8 | data[11];
    float kousyou_remaining_f = ((int)kousyou_remaining * 10) / 1000.0;
    uint16_t cycle = data[12] << 8 | data[13];
    uint16_t product_date = data[14] << 8 | data[15];
    uint16_t equilibrium = data[16] << 8 | data[17];
    uint16_t equilibrium_high = data[18] << 8 | data[19];
    uint16_t protect_status = data[20] << 8 | data[21];
    std::bitset<16> protect_status_high = data[20] << 8 | data[21];

    bool monomer_overvoltage_protection = protect_status_high[0];
    bool monomer_undervoltage_protection = protect_status_high[1];
    bool group_overvoltage_protection = protect_status_high[2];
    bool group_undervoltage_protection = protect_status_high[3];
    bool overheat_protection = protect_status_high[4];
    bool cold_protection_charging = protect_status_high[5];
    bool overheat_protection_discharge = protect_status_high[6];
    bool cold_protection_discharge = protect_status_high[7];
    bool charge_overcurrent_protection = protect_status_high[8];
    bool discharge_overcurrent_protection = protect_status_high[9];
    bool short_circuit_protection = protect_status_high[10];
    bool ic_error = protect_status_high[11];
    bool softwarelock_mos = protect_status_high[12];

    uint8_t software_version = data[22];
    uint8_t rsoc = data[23];
    uint8_t fet_status = data[24];
    uint8_t battery_num = data[25];
    uint8_t ntc_num = data[26];
    uint16_t ntc_contents = (data[27] << 8 | data[28]) - 2731;

    ROS_INFO("SerialCommunication::kousyou_remaining_f: %f", kousyou_remaining_f);
    ROS_INFO("SerialCommunication::cycle: %d", cycle);
    ROS_INFO("SerialCommunication::product_date: %d", product_date);
    ROS_INFO("SerialCommunication::equilibrium: %d", equilibrium);
    ROS_INFO("SerialCommunication::equilibrium_high: %d", equilibrium_high);
    ROS_INFO("SerialCommunication::protect_status: %d", protect_status);
    ROS_INFO("SerialCommunication::software_version: %d", software_version);
    ROS_INFO("SerialCommunication::rsoc: %d", rsoc);
    ROS_INFO("SerialCommunication::fet_status: %d", fet_status);
    ROS_INFO("SerialCommunication::battery_num: %d", battery_num);
    ROS_INFO("SerialCommunication::ntc_num: %d", ntc_num);
    ROS_INFO("SerialCommunication::ntc_contents: %d", ntc_contents / 10);

    ROS_INFO("--------------------------------------------------");

    // Get Each Cell Volume
  }
  else if (data[1] == 0x04)
  {
    ROS_INFO("SerialCommunication::serialParse::Each Volume");
    uint8_t cell_num = data[3];
    // Check Number of Cells もしセル数が4ではなかったらワーニング
    if (cell_num == 0x08)
    {
      uint16_t cell_volume_16_1 = data[4] << 8 | data[5];
      uint16_t cell_volume_16_2 = data[6] << 8 | data[7];
      uint16_t cell_volume_16_3 = data[8] << 8 | data[9];
      uint16_t cell_volume_16_4 = data[10] << 8 | data[11];

      double cell_volume_1 = ((int)cell_volume_16_1) / 1000.0;
      double cell_volume_2 = ((int)cell_volume_16_2) / 1000.0;
      double cell_volume_3 = ((int)cell_volume_16_3) / 1000.0;
      double cell_volume_4 = ((int)cell_volume_16_4) / 1000.0;

      ROS_INFO("SerialCommunication::Voltage 1: %f", cell_volume_1);
      ROS_INFO("SerialCommunication::Voltage 2: %f", cell_volume_2);
      ROS_INFO("SerialCommunication::Voltage 3: %f", cell_volume_3);
      ROS_INFO("SerialCommunication::Voltage 4: %f", cell_volume_4);
    }
    else
    {
      ROS_WARN("SerialCommunication::serialParse::Vattery Cell is only %x", cell_num);
      return 1;
    }
  }
  return 0;
}

int SerialCommunication::serialSend(uint8_t* data)
{
  // ROS_INFO("SerialCommunication::serialSend");
  write(fd_, (char*)data, sizeof(data) / sizeof(*data));
  return 0;
}

//シリアル受信：データを読み込んで0xDDから始まり0x77で終わればserial_parseを呼ぶ
int SerialCommunication::serialRead()
{
  int req_size = 1;        // 1バイト読み込む
  int available_size = 0;  //バッファに溜まっているデータ数を取得する
  ioctl(fd_, FIONREAD, &available_size);

  if (available_size == 0)
  {
    return 1;
  }

  // ROS_INFO("SerialCommunication::avaiable_size: %d", available_size);

  //[TODO ]バッファにデータが溜まりすぎていたら消す

  //受信待機が発生しないようにする（いらんかも)
  int read_size;
  if (req_size > available_size)
  {
    read_size = read(fd_, &buf_, available_size);
    // ROS_INFO("SerialCommunication::[%02x]", buf_);
  }
  else
  {
    read_size = read(fd_, &buf_, req_size);
    // ROS_INFO("SerialCommunication::[%02x]", buf_);
  }

  // 0xDDがデータ開始バイト
  if (buf_ == 0xdd)
  {
    data_start_flag_ = true;
    // ROS_INFO("SerialCommunication::data_start");
  }
  // 0X77がデータの終了バイト
  if (data_start_flag_ && buf_ == 0x77)
  {
    data_end_flag_ = true;
    // ROS_INFO("SerialCommunication::data_end");
  }

  //最後は格納してからデータをクリアする
  if (data_end_flag_)
  {
    data_vector_.push_back(buf_);
    serialParse(data_vector_);
    data_vector_.clear();
    data_start_flag_ = false;
    data_end_flag_ = false;
  }
  // 0xDDから始まっていればデータに格納する
  if (data_start_flag_)
  {
    data_vector_.push_back(buf_);
  }

  return 0;
}

//
void SerialCommunication::updateBatteryStatus(std::vector<double>& battery_status)
{
  requsetLoop();
  battery_status[BATTERY_REMAINING] = battery_remaining_;
  battery_status[BATTERY_VOLTAGE] = battery_voltage_;
  battery_status[CHARGE_CURRENT] = charge_current_;
}

// リクエストを書く
void SerialCommunication::requsetLoop()
{
  requsetBatteryStatus();
  // requestEachBatteryVoltage();
}

// バッテリの総電圧、電流、残り容量をリクエストするパケットの送信
int SerialCommunication::requsetBatteryStatus()
{
  uint8_t request_battery_status[7];
  request_battery_status[0] = 0xDD;
  request_battery_status[1] = 0xA5;
  request_battery_status[2] = 0x03;
  request_battery_status[3] = 0x00;
  request_battery_status[4] = 0xFF;
  request_battery_status[5] = 0xFD;
  request_battery_status[6] = 0x77;

  if (serialSend(request_battery_status))
  {
    ROS_WARN("SerialCommunication::requestBatteryStatus::cannnot send data");
  }
  return 0;
}

// 各セルの電圧をリクエストするパケットの送信
int SerialCommunication::requestEachBatteryVoltage()
{
  uint8_t request_battery_voltage[7];
  request_battery_voltage[0] = 0xDD;
  request_battery_voltage[1] = 0xA5;
  request_battery_voltage[2] = 0x04;
  request_battery_voltage[3] = 0x00;
  request_battery_voltage[4] = 0xFF;
  request_battery_voltage[5] = 0xFC;
  request_battery_voltage[6] = 0x77;

  if (serialSend(request_battery_voltage))
  {
    ROS_WARN("SerialCommunication::requestEachBatteryVoltage::cannnot send data");
  }

  return 0;
}

// シリアル通信の開始
int SerialCommunication::serialInitialize()
{
  struct termios tio;
  memset(&tio, 0, sizeof(tio));
  tio.c_cflag = CS8 | CLOCAL | CREAD;
  tio.c_cc[VTIME] = 100;

  int serial_open_count;
  for (serial_open_count = 0; serial_open_count < 5; serial_open_count++)
  {
    if (serialOpen() < 0)
    {
      ROS_WARN("SerialCommunication::Could not open port: /dev/ttyUSB0");
    }
    else
    {
      ROS_INFO("SerialCommunication::Open port: /dev/ttyUSB0");
      is_usb_connected_ = 1;

      tio.c_cflag += CREAD;   // 受信有効
      tio.c_cflag += CLOCAL;  // ローカルライン（モデム制御なし）
      tio.c_cflag += CS8;     // データビット:8bit
      tio.c_cflag += 0;       // ストップビット:1bit
      tio.c_cflag += 0;       // パリティ:None
      cfsetispeed(&tio, BAUD_RATE);
      cfsetospeed(&tio, BAUD_RATE);
      cfmakeraw(&tio);                // RAWモード
      tcsetattr(fd_, TCSANOW, &tio);  // デバイスに設定を行う
      ioctl(fd_, TCSETS, &tio);       // ポートの設定を有効にする
      // serialRead();
      return 0;
    }
  }
  ROS_WARN("SerialCommunication::serialInitialize: failed");
  is_usb_connected_ = 0;
  return 1;
}

// ポートを開く
int SerialCommunication::serialOpen()
{
  fd_ = open(SERIAL_PORT, O_RDWR);
  return fd_;
}

// ポートを閉じる
int SerialCommunication::serialClose()
{
  close(fd_);
}

// 疎通確認
int SerialCommunication::serialPing()
{
  return 0;
}

// [TODO] fdの再初期化
int SerialCommunication::serialReconnect()
{
  // close(fd_);
  // serialInitialize();
  return 0;
}

SerialCommunication::SerialCommunication()
{
  battery_remaining_ = 0.0;
  battery_voltage_ = 0.0;
  charge_current_ = 0.0;
  data_count_ = 0;
  data_start_flag_ = false;
  data_end_flag_ = false;
}

SerialCommunication::~SerialCommunication()
{
  serialClose();
}
