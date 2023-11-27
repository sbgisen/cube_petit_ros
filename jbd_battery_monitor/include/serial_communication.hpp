#ifndef SERIAL_COMMUNICATION
#define SERIAL_COMMUNICATION

#include <cstdlib>
#include <std_msgs/String.h>
#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <bitset>
#include <errno.h>  // Error integer and strerror() function

#define SERIAL_PORT "/dev/ttyJBDBM"
#define BAUD_RATE B9600  // RS232C通信ボーレート
#define BUFF_SIZE 4096   // 適当

class Serial_Parse;

class SerialCommunication
{
private:
  bool data_start_flag_;
  bool data_end_flag_;
  int data_count_;

  int fd_;
  uint8_t buf_;                       // 2byte
  std::vector<uint8_t> data_vector_;  // 0xDDから始まって0x77で終わるデータを格納する
  double battery_remaining_;
  double battery_voltage_;
  double charge_current_;

  enum Params
  {
    BATTERY_REMAINING,
    BATTERY_VOLTAGE,
    CHARGE_CURRENT
  };

public:
  int is_usb_connected_;  // 0: no connection 1: connected  2: reconnecting

  SerialCommunication();
  // SerialCommunication(SerialCommunication* serial_communication);
  ~SerialCommunication();
  int serialInitialize();
  int serialClose();
  int serialOpen();
  int serialPing();
  int serialReconnect();
  int serialSend(uint8_t* data);
  int serialRead();
  void updateBatteryStatus(std::vector<double>&);
  int requsetBatteryStatus();
  int requestEachBatteryVoltage();
  void requsetLoop();
  int serialParse(std::vector<uint8_t>& data);
};

#endif  // SERIAL_COMMUNICATION
