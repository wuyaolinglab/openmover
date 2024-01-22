/*
 * This file is part of lslidar_ch driver.
 *
 * The driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the driver.  If not, see <http://www.gnu.org/licenses/>.
 *
 *     Input -- base class used to access the data independently of
 *              its source
 *
 *     InputSocket -- derived class reads live data from the device
 *              via a UDP socket
 *
 *     InputPCAP -- derived class provides a similar interface from a
 *              PCAP dump
 */

#ifndef __LSLIDAR_INPUT_H_
#define __LSLIDAR_INPUT_H_

#include <unistd.h>
#include <stdio.h>
#include <pcap.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <lslidar_msgs/LslidarPacket.h>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <signal.h>
#include <sensor_msgs/TimeReference.h>
#include <std_msgs/Int8.h>

namespace lslidar_driver
{
static uint16_t PACKET_SIZE_input = 1206;
static uint16_t MSOP_DATA_PORT_NUMBER = 2368;   // lslidar default data port on PC
static uint16_t DIFOP_DATA_PORT_NUMBER = 2369;  // lslidar default difop data port on PC
/**
 *  从在线的网络数据或离线的网络抓包数据（pcap文件）中提取出lidar的原始数据，即packet数据包
 * @brief The Input class,
     *
     * @param private_nh  一个NodeHandled,用于通过节点传递参数
     * @param port
     * @returns 0 if successful,
     *          -1 if end of file
     *          >0 if incomplete packet (is this possible?)
 */
class Input
{
public:
  Input(ros::NodeHandle private_nh, uint16_t port);

  virtual ~Input()
  {
  }

  virtual int getPacket(lslidar_msgs::LslidarPacketPtr& packet) = 0;

  int getRpm(void);
  int getReturnMode(void);
  bool getUpdateFlag(void);
  void clearUpdateFlag(void);
  void UDP_order(const std_msgs::Int8 msg);
  void UDP_difop();
protected:
  ros::NodeHandle private_nh_;
  uint16_t port_;
  std::string devip_str_;
  std::string lidar_name;
  int cur_rpm_;
  int return_mode_;
  bool npkt_update_flag_;
  bool add_multicast;
  std::string group_ip;
  int UDP_PORT_NUMBER_DIFOP;
	int socket_id_difop;
  int sockfd_;
  std::string devip_str_difop;
};

/** @brief Live lslidar input from socket. */
class InputSocket : public Input
{
public:
  InputSocket(ros::NodeHandle private_nh, uint16_t port = MSOP_DATA_PORT_NUMBER);

  virtual ~InputSocket();

  //virtual int getPacket(lslidar_ch_msgs::LslidarPacket* pkt, const double time_offset);
  virtual int getPacket(lslidar_msgs::LslidarPacketPtr& packet);

private:
private:
  
  in_addr devip_;
  in_addr devip_difop;
  //struct ip_mreq group;

};
class InputPCAP : public Input
{
public:
    InputPCAP(ros::NodeHandle private_nh,uint16_t port = MSOP_DATA_PORT_NUMBER, double packet_rate = 0.0,
            std::string filename="",bool read_once = false,bool read_fast = false,double repeat_delay = 0.0);
    virtual ~InputPCAP();
    virtual int getPacket(lslidar_msgs::LslidarPacketPtr& pkt);
private:

    ros::Rate packet_rate_;
    std::string filename_;
    pcap_t *pcap_;
    bpf_program pcap_packet_filter_;
    char errbuf_[PCAP_ERRBUF_SIZE];
    bool empty_;
    bool read_once_;
    bool read_fast_;
    double repeat_delay_;
};

}

#endif  // __LSLIDAR_INPUT_H
