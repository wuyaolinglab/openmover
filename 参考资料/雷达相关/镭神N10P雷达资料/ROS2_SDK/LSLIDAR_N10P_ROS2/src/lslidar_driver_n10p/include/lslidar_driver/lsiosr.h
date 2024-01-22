/*******************************************************
@company: Copyright (C) 2021, Leishen Intelligent System
@product: LSM10_N10
@filename: lsiosr.cpp
@brief:
@version:       date:       author:     comments:
@v1.0           22-10-24      li          new
*******************************************************/
#ifndef LSIOSR_H
#define LSIOSR_H

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <fstream>
#include <iostream>

//波特率
#define     BAUD_230400     230400
#define     BAUD_460800     460800
#define     BAUD_500000     500000
#define     BAUD_921600     921600

//奇偶校验位
#define     PARITY_ODD    	'O' //奇数
#define     PARITY_EVEN   	'E' //偶数
#define     PARITY_NONE   	'N' //无奇偶校验位

//停止位
#define     STOP_BIT_1     	1
#define     STOP_BIT_2     	2

//数据位
#define     DATA_BIT_7     	7
#define     DATA_BIT_8     	8

namespace lslidar_driver
{
class LSIOSR{
public:
  static LSIOSR* instance(std::string name, int speed, int fd = 0);

  ~LSIOSR();

  /* 从串口中读取数据 */
  int read(unsigned char *buffer, int length, int timeout = 30);

  /* 向串口传数据 */
  int send(const char* buffer, int length, int timeout = 30);

  /* Empty serial port input buffer */
  void flushinput();

  /* 串口初始化 */
  int init();

  int close();

  /* 获取串口号 */
  std::string getPort();

  /* 设置串口号 */
  int setPortName(std::string name);

private:
  LSIOSR(std::string name, int speed, int fd);

  int waitWritable(int millis);
  int waitReadable(int millis);

  /* 串口配置的函数 */
  int setOpt(int nBits, uint8_t nEvent, int nStop);

  std::string port_;
  int baud_rate_;

  int fd_;
};
}
#endif

