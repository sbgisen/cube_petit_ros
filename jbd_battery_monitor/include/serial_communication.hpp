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

#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUD_RATE    B9600                // RS232C通信ボーレート
#define BUFF_SIZE    4096                 // 適当

class Serial_Parse;

class Serial_Communication{

private:
  bool data_start_flag;
  bool data_end_flag;
  int data_count;

  int fd;
  uint8_t buf;                        //2byte
  std::vector<uint8_t> data_vector;   //0xDDから始まって0x77で終わるデータを格納する
  double battery_remaining;
  double battery_voltage;
  double charge_current;

  enum Params {
    BATTERY_REMAINING,
    BATTERY_VOLTAGE,
    CHARGE_CURRENT
  };

public:
  int is_usb_connected;   // 0: no connection 1: connected  2: reconnecting

  Serial_Communication();
  // Serial_Communication(Serial_Communication* serial_communication);
  ~Serial_Communication();
  int serial_initialize();
  int serial_close();
  int serial_open();
  int serial_ping();
  int serial_reconnect();
  int serial_send(uint8_t *data);
  int serial_read();
  void updateBatteryStatus(std::vector<double>&);
  int requsetBatteryStatus();
  int requestEachBatteryVoltage();
  void requset_loop();
  int serial_parse(std::vector<uint8_t>& data);


};

#endif //SERIAL_COMMUNICATION


