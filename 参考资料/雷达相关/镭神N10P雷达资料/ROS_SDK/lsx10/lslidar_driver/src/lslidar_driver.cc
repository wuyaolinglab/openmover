/*
 * This file is part of lslidar driver.
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
 */

#include <string>
#include <cmath>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <fstream>
#include <iostream>

#include <lslidar_driver/lslidar_driver.h>

namespace lslidar_driver
{

	LslidarDriver::LslidarDriver(
		ros::NodeHandle &n, ros::NodeHandle &pn) : nh(n),
												   pnh(pn)
	{
		return;
	}

	LslidarDriver::~LslidarDriver()
	{
		return;
	}

	void LslidarDriver::initParam() // 初始化驱动参数
	{
		pubscan_thread_ = new boost::thread(boost::bind(&LslidarDriver::pubScanThread, this));				// 创建新线程
		difop_switch = nh.subscribe<std_msgs::Int8>("lslidar_order", 1, &LslidarDriver::lidar_order, this); // 指令输入
		is_start = true;
		pnh.param<std::string>("lidar_name", lidar_name, "M10");
		pnh.param<std::string>("frame_id", frame_id, "laser_link");
		pnh.param<std::string>("scan_topic", scan_topic, "/scan");
		pnh.param<std::string>("interface_selection", interface_selection, "net");
		pnh.param<std::string>("pointcloud_topic", pointcloud_topic, "lslidar_point_cloud");
		pnh.param<bool>("pubScan", pubScan, true);
		pnh.param<bool>("use_gps_ts", use_gps_ts, false);
		pnh.param<bool>("compensation", compensation, false);
		pnh.param<bool>("pubPointCloud2", pubPointCloud2, false);
		pnh.param<bool>("high_reflection", high_reflection, false);
		pnh.param<bool>("n10p_double_echo", n10p_double_echo, false);
		pnh.param<double>("min_range", min_range, 0.3);
		pnh.param<double>("max_range", max_range, 100.0);
		pnh.param<double>("angle_disable_min", angle_disable_min, 0.0);
		pnh.param<double>("angle_disable_max", angle_disable_max, 0.0);

		pnh.param("truncated_mode", truncated_mode_, 0);
   		pnh.param<std::vector<int>>("disable_min", disable_angle_min_range, {0});
    	pnh.param<std::vector<int>>("disable_max", disable_angle_max_range, {0});
		angle_able_min = 0;
		angle_able_max = 360;

		count_num = 0;

		scan_points_.resize(6000);

		if (lidar_name == "M10")
		{
			use_gps_ts = false;
			PACKET_SIZE = 92;
			package_points = 42;
			data_bits_start = 6;
			degree_bits_start = 2;
			rpm_bits_start = 4;
			baud_rate_ = 460800;
			points_size_ = 1008;
		}
		else if (lidar_name == "M10_P")
		{
			PACKET_SIZE = 160;
			package_points = 70;
			data_bits_start = 8;
			degree_bits_start = 4;
			rpm_bits_start = 6;
			baud_rate_ = 500000;
			points_size_ = 2000;
		}
		else if (lidar_name == "M10_PLUS")
		{
			PACKET_SIZE = 104;
			package_points = 41;
			data_bits_start = 8;
			degree_bits_start = 4;
			rpm_bits_start = 6;
			points_size_ = 5000;
			baud_rate_ = 921600;
		}
		else if (lidar_name == "M10_GPS")
		{
			PACKET_SIZE = 102;
			package_points = 42;
			data_bits_start = 6;
			degree_bits_start = 2;
			rpm_bits_start = 4;
			baud_rate_ = 460800;
			points_size_ = 1008;
		}
		else if (lidar_name == "N10")
		{
			PACKET_SIZE = 58;
			package_points = 16;
			data_bits_start = 7;
			degree_bits_start = 5;
			end_degree_bits_start = 55;
			baud_rate_ = 230400;
			points_size_ = 2000;
			use_gps_ts = false;
			compensation = false;
		}
		else if (lidar_name == "N10_P")
		{
			PACKET_SIZE = 108;
			package_points = 16;
			data_bits_start = 7;
			degree_bits_start = 5;
			end_degree_bits_start = 105;
			baud_rate_ = 460800;
			points_size_ = 2000;
			use_gps_ts = false;
			compensation = false;
		}
		else if (lidar_name == "M10_DOUBLE")
		{
			PACKET_SIZE = 300;
			package_points = 70;
			data_bits_start = 8;
			degree_bits_start = 4;
			rpm_bits_start = 6;
			points_size_ = 3000;
			baud_rate_ = 921600;
		}
		else if (lidar_name == "L10")
		{
			PACKET_SIZE = 58;
			package_points = 16;
			data_bits_start = 7;
			degree_bits_start = 5;
			end_degree_bits_start = 55;
			baud_rate_ = 230400;
			points_size_ = 2000;
			use_gps_ts = false;
			compensation = false;
		}
		ROS_INFO_STREAM("Lidar is " << lidar_name);
		return;
	}

	void LslidarDriver::lidar_difop() // 获取雷达设备包
	{
		if (lidar_name == "L10" || lidar_name == "N10" || lidar_name == "N10_P")
			return;
		if (interface_selection == "net")
			msop_input_->UDP_difop();
		else
		{
			for (int k = 0; k < 10; k++)
			{
				unsigned char data[188] = {0x00};
				data[0] = 0xA5;
				data[1] = 0x5A;
				data[2] = 0x55;
				data[184] = 0x08;
				data[185] = 0x01;
				data[186] = 0xFA;
				data[187] = 0xFB;
				int rtn = serial_->send((const char *)data, 188);
				if (rtn < 0)
					printf("start scan error !\n");
				else
					return;
			}
		}
		return;
	}

	void LslidarDriver::lidar_order(const std_msgs::Int8 msg) // 发送控制雷达指令
	{
		if (lidar_name == "L10")
			return;
		int i = msg.data;
		if (i == 0)
			is_start = false;
		else
			is_start = true;
		if (interface_selection == "net")
			msop_input_->UDP_order(msg);
		else
		{
			int i = msg.data;
			for (int k = 0; k < 10; k++)
			{
				int rtn;
				unsigned char data[188] = {0x00};
				data[0] = 0xA5;
				data[1] = 0x5A;
				data[2] = 0x55;
				data[186] = 0xFA;
				data[187] = 0xFB;

				if (lidar_name == "M10" || lidar_name == "M10_GPS" || lidar_name == "M10_P" || lidar_name == "M10_DOUBLE")
				{
					if (i <= 1) // 控制雷达启停
					{
						data[184] = 0x01;
						data[185] = char(i);
					}
					else if (i == 2) // 雷达点云不滤波
					{
						data[181] = 0x0A;
						data[184] = 0x06;
						if (is_start)
							data[185] = 0x01;
					}
					else if (i == 3) // 雷达点云正常滤波
					{
						data[181] = 0x0B;
						data[184] = 0x06;
						if (is_start)
							data[185] = 0x01;
					}
					else if (i == 4) // 雷达近距离滤波
					{
						data[181] = 0x0C;
						data[184] = 0x06;
						if (is_start)
							data[185] = 0x01;
					}
					else if (i == 100) // 接收设备包
					{
						data[184] = 0x08;
						data[185] = 0x01;
					}
					else
						return;
				}
				else if (lidar_name == "M10_PLUS")
				{
					data[184] = 0x0A;
					data[185] = 0x01;
					if (i == 5)
					{
						data[141] = 0x01;
						data[142] = 0x2c;
					}
					else if (i == 6)
					{
						data[141] = 0x01;
						data[142] = 0x68;
					}
					else if (i == 8)
					{
						data[141] = 0x01;
						data[142] = 0xe0;
					}
					else if (i == 10)
					{
						data[141] = 0x02;
						data[142] = 0x58;
					}
					else if (i == 12)
					{
						data[141] = 0x02;
						data[142] = 0xd0;
					}
					else if (i == 15)
					{
						data[141] = 0x03;
						data[142] = 0x84;
					}
					else if (i == 20)
					{
						data[141] = 0x04;
						data[142] = 0xb0;
					}
					else if (i <= 1) // 控制雷达启停
					{
						data[184] = 0x01;
						data[185] = char(i);
					}
					else if (i == 100) // 接收设备包
					{
						data[184] = 0x08;
						data[185] = 0x01;
					}
					else
						return;
				}
				else if (lidar_name == "N10" || lidar_name == "N10_P")
				{
					if (i <= 1) // 控制雷达启停
					{
						data[185] = char(i);
						data[184] = 0x01;
					}
					else if (i >= 6 && i <= 12)
					{
						data[172] = char(i);
						data[184] = 0x0a;
						data[185] = 0X01;
					}
					else
						return;
				}
				rtn = serial_->send((const char *)data, 188);
				if (rtn < 0)
					printf("start scan error !\n");
				else
				{
					if (i == 1)
						usleep(1000000); // 1.0s
					if (i == 0)
						is_start = false;
					if (i == 1)
						is_start = true;
					return;
				}
			}
			return;
		}
	}

	void LslidarDriver::open_serial() // 打开串口
	{
		int code = 0;
		std::string port = "/dev/ ";
		pnh.param("serial_port", serial_port_, port);
		serial_ = LSIOSR::instance(serial_port_, baud_rate_);
		code = serial_->init();
		if (code != 0)
		{
			printf("open_port %s ERROR !\n", serial_port_.c_str());
			ros::shutdown();
			exit(0);
		}
		printf("open_port %s OK !\n", serial_port_.c_str());
	}

	bool LslidarDriver::createRosIO() // 网口初始化
	{
		pnh.param<int>("device_port", UDP_PORT_NUMBER, 2368);
		ROS_INFO_STREAM("Opening UDP socket: port " << UDP_PORT_NUMBER);
		// ROS diagnostics
		diagnostics.setHardwareID("Lslidar");

		const double diag_freq = 12 * 24;
		diag_max_freq = diag_freq;
		diag_min_freq = diag_freq;
		ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

		using namespace diagnostic_updater;
		diag_topic.reset(new TopicDiagnostic(
			"lslidar_packets", diagnostics,
			FrequencyStatusParam(&diag_min_freq, &diag_max_freq, 0.1, 10),
			TimeStampStatusParam()));

		int hz = 10;
		if (lidar_name == "M10_P")
			hz = 12;
		else if (lidar_name == "M10_PLUS")
			hz = 20;

		double packet_rate = hz * 24;
		pnh.param("pcap", dump_file, std::string(""));
		if (dump_file != "")
		{
			msop_input_.reset(new lslidar_driver::InputPCAP(pnh, UDP_PORT_NUMBER, packet_rate, dump_file));
		}
		else
		{
			msop_input_.reset(new lslidar_driver::InputSocket(pnh, UDP_PORT_NUMBER));
		}

		return true;
	}

	int LslidarDriver::getScan(std::vector<ScanPoint> &points, ros::Time &scan_time, float &scan_duration) // 获取数据和相对时间
	{
		boost::unique_lock<boost::mutex> lock(mutex_);
		points.assign(scan_points_bak_.begin(), scan_points_bak_.end());
		scan_time = pre_time_;
		scan_duration = (time_ - pre_time_).toSec();
		return 1;
	}

	uint64_t LslidarDriver::get_gps_stamp(struct tm t) // 转化GPS时间
	{
		uint64_t ptime = static_cast<uint64_t>(timegm(&t));
		return ptime;
	}

	bool LslidarDriver::initialize() // 根据连接方法初始化
	{
		initParam();
		if (interface_selection == "net")
		{
			if (!createRosIO())
			{
				ROS_ERROR("Cannot create all ROS IO...");
				return false;
			}
		}
		else
		{
			pnh.param<std::string>("in_file_name", in_file_name, "");
			ROS_INFO_STREAM("Opening PCAP file " << in_file_name);
			if (in_file_name == "")
				open_serial();
			else
			{
				std::ifstream file_reader(in_file_name);
				if (!file_reader.is_open())
				{
					ROS_ERROR("Cannot open the file");
					return false;
				}
			}
		}
		if (pubScan)
			pub_ = nh.advertise<sensor_msgs::LaserScan>(scan_topic, 3);
		if (pubPointCloud2)
			pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>(pointcloud_topic, 10);
		ROS_INFO("Initialised lslidar without error");
		return true;
	}

	void LslidarDriver::recvThread_crc(int &count, int &link_time) // 获取串口数据时检测是否超时
	{
		if (count <= 0)
			link_time++;
		else
			link_time = 0;

		if (link_time > 150)
		{
			ROS_ERROR("long time lose data!\n");
			serial_->close();
			int ret = serial_->init();
			if (ret < 0)
			{
				ROS_WARN("serial open fail");
				usleep(200000);
			}
			link_time = 0;
		}
	}

	int LslidarDriver::receive_data(unsigned char *packet_bytes) // 获取串口数据
	{
		int link_time = 0;
		int len_H = 0;
		int len_L = 0;
		int len = 0;
		int count_2 = 0;
		int count = 0;
		while (count <= 0)
		{
			count = serial_->read(packet_bytes, 1);
			LslidarDriver::recvThread_crc(count, link_time);
		}
		if (packet_bytes[0] != 0xA5)
			return 0;

		while (count_2 <= 0)
		{
			count_2 = serial_->read(packet_bytes + count, 1);
			if (count_2 >= 0)
				count += count_2;
			LslidarDriver::recvThread_crc(count_2, link_time);
		}

		count_2 = 0;
		if (packet_bytes[1] != 0x5A)
			return 0;
		while (count_2 <= 0)
		{
			count_2 = serial_->read(packet_bytes + count, 2);
			if (count_2 >= 0)
				count += count_2;
			LslidarDriver::recvThread_crc(count_2, link_time);
		}

		count_2 = 0;

		if (lidar_name == "M10")
			len = 92;
		else if (lidar_name == "M10_GPS")
			len = 102;
		else if (lidar_name == "N10_P")
			len = 108;
		else if (lidar_name == "N10" || lidar_name == "L10")
			len = packet_bytes[2];
		else
		{
			len_H = packet_bytes[2];
			len_L = packet_bytes[3];
			len = len_H * 256 + len_L;
		}
		if (lidar_name == "M10" || lidar_name == "M10_DOUBLE" || lidar_name == "M10_GPS" || lidar_name == "M10_P" || lidar_name == "M10_PLUS")
		{
			if (packet_bytes[2] == 0x55 && packet_bytes[3] == 0x00)
				len = 188;
		}
		while (count < len)
		{
			count_2 = serial_->read(packet_bytes + count, len - count);
			if (count_2 >= 0)
				count += count_2;
			LslidarDriver::recvThread_crc(count_2, link_time);
		}
		if (lidar_name == "N10" || lidar_name == "L10" || lidar_name == "N10_P")
		{
			if (packet_bytes[PACKET_SIZE - 1] != N10_CalCRC8(packet_bytes, PACKET_SIZE - 1))
				return 0;
		}
		return len;
	}

	uint8_t LslidarDriver::N10_CalCRC8(unsigned char *p, int len) // N10雷达的数据包校验
	{
		int sum = 0;
		for (int i = 0; i < len; i++)
			sum += uint8_t(p[i]);
		uint8_t crc = sum & 0xff;
		return crc;
	}

	void LslidarDriver::difop_processing(unsigned char *packet_bytes) // 处理设备包的数据
	{
		int s = packet_bytes[173];
		int z = packet_bytes[174];
		int degree_temp = s & 0x7F;
		int sign_temp = s & 0x80;
		degree_compensation = double(degree_temp * 256 + z) / 100.f;
		if (sign_temp)
			degree_compensation = -degree_compensation;
		first_compensation = false;
		printf("degree_compensation = %f\n", degree_compensation);
		return;
	}

	void LslidarDriver::data_processing(unsigned char *packet_bytes, int len) // 处理每一包的数据
	{
		double degree;
		double end_degree;
		double degree_interval = 15.0;
		boost::posix_time::ptime t1, t2;
		t1 = boost::posix_time::microsec_clock::universal_time();

		int s = packet_bytes[degree_bits_start];
		int z = packet_bytes[degree_bits_start + 1];

		degree = (s * 256 + z) / 100.f + degree_compensation;
		degree = (degree < 0) ? degree + 360 : degree;
		degree = (degree > 360) ? degree - 360 : degree;
		if (lidar_name == "N10" || lidar_name == "L10")
		{
			int s_e = packet_bytes[end_degree_bits_start];
			int z_e = packet_bytes[end_degree_bits_start + 1];

			end_degree = (s_e * 256 + z_e) / 100.f;
			end_degree = (end_degree > 360) ? end_degree - 360 : end_degree;

			if (degree > end_degree)
				degree_interval = end_degree + 360 - degree;
			else
				degree_interval = end_degree - degree;
		}

		// boost::unique_lock<boost::mutex> lock(mutex_);
		if (lidar_name == "M10_PLUS" || lidar_name == "M10_P")
		{
			PACKET_SIZE = len;
			package_points = (PACKET_SIZE - 20) / 2;
		}
		int invalidValue = 0;
		int point_len = 2;
		if (lidar_name == "N10" || lidar_name == "L10")
			point_len = 3;

		if (lidar_name == "M10_GPS" || lidar_name == "M10")
		{
			int err_data_84 = packet_bytes[84];
			int err_data_85 = packet_bytes[85];
			if ((err_data_84 * 256 + err_data_85) == 0xFFFF || packet_bytes[86] >= 0xF5)
			{
				packet_bytes[86] = 0xFF;
				packet_bytes[87] = 0xFF;
			}
		}

		for (int num = 0; num < point_len * package_points; num += point_len)
		{
			int s = packet_bytes[num + data_bits_start];
			int z = packet_bytes[num + data_bits_start + 1];
			if ((s * 256 + z) == 0xFFFF)
				invalidValue++;
		}

		if (use_gps_ts)
		{
			pTime.tm_year = packet_bytes[PACKET_SIZE - 12] + 2000 - 1900; // x+2000
			pTime.tm_mon = packet_bytes[PACKET_SIZE - 11] - 1;			  // 1-12
			pTime.tm_mday = packet_bytes[PACKET_SIZE - 10];				  // 1-31
			pTime.tm_hour = packet_bytes[PACKET_SIZE - 9];				  // 0-23
			pTime.tm_min = packet_bytes[PACKET_SIZE - 8];				  // 0-59
			pTime.tm_sec = packet_bytes[PACKET_SIZE - 7];				  // 0-59
			sub_second = (packet_bytes[PACKET_SIZE - 6] * 256 + packet_bytes[PACKET_SIZE - 5]) * 1000000 + (packet_bytes[PACKET_SIZE - 4] * 256 + packet_bytes[PACKET_SIZE - 3]) * 1000;
			sweep_end_time_gps = get_gps_stamp(pTime);
			sweep_end_time_hardware = sub_second % 1000000000;
		}
		invalidValue = package_points - invalidValue;
		if (lidar_name == "N10" || lidar_name == "L10")
			invalidValue--;
		if (invalidValue <= 1)
		{
			delete packet_bytes;
			return;
		}

		for (int num = 0; num < package_points; num++)
		{
			int s = packet_bytes[num * point_len + data_bits_start];
			int z = packet_bytes[num * point_len + data_bits_start + 1];
			int y = 0;
			if (lidar_name == "N10" || lidar_name == "L10")
				y = packet_bytes[num * point_len + data_bits_start + 2];
			int dist_temp = s & 0x7F;
			int inten_temp = s & 0x80;

			if ((s * 256 + z) != 0xFFFF)
			{
				if (lidar_name == "N10" || lidar_name == "L10")
				{
					scan_points_[idx].range = double(s * 256 + (z)) / 1000.f;
					scan_points_[idx].intensity = int(y);
				}
				else if ((lidar_name == "M10_P" || lidar_name == "M10_PLUS") && !high_reflection)
				{
					scan_points_[idx].range = double(s * 256 + (z)) / 1000.f;
					scan_points_[idx].intensity = 0;
				}
				else
				{
					scan_points_[idx].range = double(dist_temp * 256 + (z)) / 1000.f;
					if (inten_temp)
						scan_points_[idx].intensity = 255;
					else
						scan_points_[idx].intensity = 0;
				}
				if ((degree + (degree_interval / invalidValue * num)) > 360)
					scan_points_[idx].degree = degree + (degree_interval / invalidValue * num) - 360;
				else
					scan_points_[idx].degree = degree + (degree_interval / invalidValue * num);
			}
			else
				continue;
			if ((scan_points_[idx].degree < last_degree && scan_points_[idx].degree < 5 && last_degree > 355) || idx >= points_size_)
			{
				last_degree = scan_points_[idx].degree;
				count_num = idx;
				idx = 0;
				for (int k = 0; k < scan_points_.size(); k++)
				{
					if (scan_points_[k].range < min_range || scan_points_[k].range > max_range)
						scan_points_[k].range = 0;
				}
				boost::unique_lock<boost::mutex> lock(mutex_);
				scan_points_bak_.resize(scan_points_.size());
				scan_points_bak_.assign(scan_points_.begin(), scan_points_.end());
				for (int k = 0; k < scan_points_.size(); k++)
				{
					scan_points_[k].range = 0;
					scan_points_[k].degree = 0;
				}
				pre_time_ = time_;
				lock.unlock();
				pubscan_cond_.notify_one();
				time_ = ros::Time::now();
			}
			else
			{
				last_degree = scan_points_[idx].degree;
				idx++;
			}
		}
		packet_bytes = {0x00};
		if (packet_bytes)
		{
			packet_bytes = NULL;
			delete packet_bytes;
		}
	}

	void LslidarDriver::data_processing_2(unsigned char *packet_bytes, int len) // 处理每一包的数据/双回波
	{
		double degree;
		double end_degree;
		double degree_interval = 15.0;
		boost::posix_time::ptime t1, t2;
		t1 = boost::posix_time::microsec_clock::universal_time();

		int s = packet_bytes[degree_bits_start];
		int z = packet_bytes[degree_bits_start + 1];

		degree = (s * 256 + z) / 100.f + degree_compensation;
		degree = (degree < 0) ? degree + 360 : degree;
		degree = (degree > 360) ? degree - 360 : degree;
		if (lidar_name == "N10_P")
		{
			int s_e = packet_bytes[end_degree_bits_start];
			int z_e = packet_bytes[end_degree_bits_start + 1];

			end_degree = (s_e * 256 + z_e) / 100.f;
			end_degree = (end_degree > 360) ? end_degree - 360 : end_degree;

			if (degree > end_degree)
				degree_interval = end_degree + 360 - degree;
			else
				degree_interval = end_degree - degree;
		}

		// boost::unique_lock<boost::mutex> lock(mutex_);
		if (lidar_name == "M10_DOUBLE")
		{
			PACKET_SIZE = len;
			package_points = (PACKET_SIZE - 20) / 4;
		}
		int invalidValue = 0;
		int point_len = 4;
		if (lidar_name == "N10_P")
			point_len = 6;

		for (int num = 0; num < point_len * package_points; num += point_len)
		{
			int s = packet_bytes[num + data_bits_start];
			int z = packet_bytes[num + data_bits_start + 1];
			if ((s * 256 + z) == 0xFFFF)
				invalidValue++;
		}

		if (use_gps_ts && lidar_name == "M10_DOUBLE")
		{
			pTime.tm_year = packet_bytes[PACKET_SIZE - 12] + 2000 - 1900; // x+2000
			pTime.tm_mon = packet_bytes[PACKET_SIZE - 11] - 1;			  // 1-12
			pTime.tm_mday = packet_bytes[PACKET_SIZE - 10];				  // 1-31
			pTime.tm_hour = packet_bytes[PACKET_SIZE - 9];				  // 0-23
			pTime.tm_min = packet_bytes[PACKET_SIZE - 8];				  // 0-59
			pTime.tm_sec = packet_bytes[PACKET_SIZE - 7];				  // 0-59
			sub_second = (packet_bytes[PACKET_SIZE - 6] * 256 + packet_bytes[PACKET_SIZE - 5]) * 1000000 + (packet_bytes[PACKET_SIZE - 4] * 256 + packet_bytes[PACKET_SIZE - 3]) * 1000;
			sweep_end_time_gps = get_gps_stamp(pTime);
			sweep_end_time_hardware = sub_second % 1000000000;
		}
		invalidValue = package_points - invalidValue;
		if (lidar_name == "N10_P")
			invalidValue--;
		if (invalidValue <= 1)
		{
			delete packet_bytes;
			return;
		}

		for (int num = 0; num < package_points; num++)
		{
			int s = packet_bytes[num * point_len + data_bits_start];
			int z = packet_bytes[num * point_len + data_bits_start + 1];
			int y = 0;
			if (lidar_name == "N10_P")
				y = packet_bytes[num * point_len + data_bits_start + 2];

			if ((s * 256 + z) != 0xFFFF)
			{
				scan_points_[idx].range = double(s * 256 + (z)) / 1000.f;
				if (lidar_name == "N10_P")
					scan_points_[idx].intensity = int(y);
				else
					scan_points_[idx].intensity = 0;
				s = packet_bytes[num * point_len + data_bits_start + point_len / 2];
				z = packet_bytes[num * point_len + data_bits_start + point_len / 2 + 1];
				if (lidar_name == "N10_P")
					y = packet_bytes[num * point_len + data_bits_start + point_len / 2 + 2];

				scan_points_[idx + 3000].range = double(s * 256 + (z)) / 1000.f;
				if (lidar_name == "N10_P")
					scan_points_[idx + 3000].intensity = int(y);
				else
					scan_points_[idx + 3000].intensity = 0;

				if ((degree + (degree_interval / invalidValue * num)) > 360)
					scan_points_[idx].degree = degree + (degree_interval / invalidValue * num) - 360;
				else
					scan_points_[idx].degree = degree + (degree_interval / invalidValue * num);
			}
			else
				continue;
			if (((scan_points_[idx].degree < last_degree && scan_points_[idx].degree < 5 && last_degree > 355) || idx >= points_size_) && idx > 10)
			{
				last_degree = scan_points_[idx].degree;
				count_num = idx;
				idx = 0;
				for (int k = 0; k < count_num; k++)
				{
					if (angle_able_max > 360)
					{
						if ((360 - scan_points_[k].degree) > (angle_able_max - 360) && (360 - scan_points_[k].degree) < angle_able_min)
						{
							scan_points_[k].range = 0;
							scan_points_[k + 3000].range = 0;
						}
					}
					else
					{
						if ((360 - scan_points_[k].degree) > angle_able_max || (360 - scan_points_[k].degree) < angle_able_min)
						{
							scan_points_[k].range = 0;
							scan_points_[k + 3000].range = 0;
						}
					}
					if (scan_points_[k].range < min_range || scan_points_[k].range > max_range)
						scan_points_[k].range = 0;
					if (scan_points_[k + 3000].range < min_range || scan_points_[k + 3000].range > max_range)
						scan_points_[k + 3000].range = 0;
				}
				boost::unique_lock<boost::mutex> lock(mutex_);
				scan_points_bak_.resize(scan_points_.size());
				scan_points_bak_.assign(scan_points_.begin(), scan_points_.end());
				for (int k = 0; k < scan_points_.size(); k++)
				{
					scan_points_[k].range = 0;
					scan_points_[k].degree = 0;
				}
				pre_time_ = time_;
				lock.unlock();
				pubscan_cond_.notify_one();
				time_ = ros::Time::now();
			}
			else
			{
				last_degree = scan_points_[idx].degree;
				idx++;
			}
		}
		packet_bytes = {0x00};
		if (packet_bytes)
		{
			packet_bytes = NULL;
			delete packet_bytes;
		}
	}

	void LslidarDriver::pubScanThread() // 发布scan或者pointcloud2话题
	{
		bool wait_for_wake = true;
		boost::unique_lock<boost::mutex> lock(pubscan_mutex_);

		while (ros::ok())
		{
			while (wait_for_wake)
			{
				pubscan_cond_.wait(lock);
				wait_for_wake = false;
			}
			if (lidar_name == "N10_P" || lidar_name == "M10_DOUBLE")
			{
				if (pubScan)
				{
					std::vector<ScanPoint> points;
					ros::Time start_time;
					float scan_time;
					this->getScan(points, start_time, scan_time);
					int scan_num;
					if(n10p_double_echo == false)
					{
						scan_num = count_num ;
					}
					else
					{
						scan_num = count_num * 2;
					}
					//scan_num=points_size_;
					sensor_msgs::LaserScan msg;
					msg.header.frame_id = frame_id;
					if (use_gps_ts)
					{
						msg.header.stamp = ros::Time(sweep_end_time_gps, sweep_end_time_hardware);
					}
					else
					{
						msg.header.stamp = start_time;
					}

					msg.angle_min = - M_PI;
					msg.angle_max =  M_PI;
					msg.angle_increment = 2 * M_PI / (double)(scan_num);
					msg.range_min = min_range;
					msg.range_max = max_range;
					msg.ranges.resize(scan_num);
					msg.intensities.resize(scan_num);
					// msg.scan_time = scan_time;
					// msg.time_increment = scan_time / (double)(count_num);

					for (int k = 0; k < scan_num; k++)
					{
						msg.ranges[k] = std::numeric_limits<float>::infinity();
						msg.intensities[k] = 0;
					}

					for (int i = 0; i < count_num; i++)
					{
						int point_idx;
						double dist;
						if(n10p_double_echo == false)
						{
							point_idx = round((360 - points[i].degree) * count_num / 360);
							if (points[i].range == 0.0)
							{
								msg.ranges[point_idx] = std::numeric_limits<float>::infinity();
								msg.intensities[point_idx] = 0;
							}
							else
							{
								dist = points[i].range;
								msg.ranges[point_idx] = (float)dist;
								msg.intensities[point_idx] = points[i].intensity;
							}	
						}
						else
						{
							point_idx = round((360 - points[i].degree) * count_num / 360);
							if (points[i].range == 0.0)
							{
								msg.ranges[point_idx*2] = std::numeric_limits<float>::infinity();
								msg.intensities[point_idx*2] = 0;
							}
							else
							{
								dist = points[i].range;
								msg.ranges[point_idx*2] = (float)dist;
								msg.intensities[point_idx*2] = points[i].intensity;
							}	
							if (points[i + 3000].range == 0.0)
							{
								msg.ranges[point_idx*2+1] = std::numeric_limits<float>::infinity();
								msg.intensities[point_idx*2+1] = 0;
							}
							else
							{
								double dist = points[i + 3000].range;
								msg.ranges[point_idx*2+1] = (float)dist;
								msg.intensities[point_idx*2+1] = points[i + 3000].intensity;
							}	
						}


						if(truncated_mode_==1)
				        {
				            for (int j = 0; j < disable_angle_max_range.size(); ++j) {
				            if ((points[i].degree >= (disable_angle_min_range[j]) ) && (points[i].degree <= (disable_angle_max_range[j]))) {
				                msg.ranges[point_idx] = std::numeric_limits<float>::infinity();
				                msg.intensities[point_idx] = 0;
				              }
				            }
				        }
				        if(n10p_double_echo == false)
				        {
				        	if(msg.intensities[point_idx] <=5 && msg.intensities[point_idx] >0)
				        	{
				            	msg.ranges[point_idx] = std::numeric_limits<float>::infinity();
				            	msg.intensities[point_idx] = 0;
				        	}
				        }
				        
					}
					pub_.publish(msg);
				}
				if (pubPointCloud2)
				{
					std::vector<ScanPoint> points;
					ros::Time start_time;
					float scan_time;
					this->getScan(points, start_time, scan_time);
					VPointCloud::Ptr point_cloud(new VPointCloud());
					if (use_gps_ts)
					{
						start_time = ros::Time(sweep_end_time_gps, sweep_end_time_hardware);
					}
					double timestamp = start_time.toSec();
					point_cloud->header.stamp = static_cast<uint64_t>(timestamp * 1e6);
					point_cloud->header.frame_id = frame_id;
					point_cloud->height = 1;
					for (uint16_t i = 0; i < count_num; i++)
					{
						double degree = 360.0 - points[i].degree;
						bool pass_point = false;
						if (angle_able_max < 360)
						{
							if (degree < angle_able_min || degree > angle_able_max)
								pass_point = true;
						}
						else
						{
							if (degree < angle_able_min && degree > (angle_able_max - 360))
								pass_point = true;
						}
						if (points[i].range < 0.001)
							pass_point = true;
						if (!pass_point)
						{
							VPoint point;
							int point_idx = round(degree * count_num / 360);
							point.timestamp = timestamp - point_idx * (scan_time / count_num);
							point.x = points[i].range * cos(M_PI / 180 * points[i].degree);
							point.y = -points[i].range * sin(M_PI / 180 * points[i].degree);
							point.z = 0;
							point.intensity = points[i].intensity;
							point_cloud->points.push_back(point);
							++point_cloud->width;
						}
						if (points[i + 3000].range < 0.001)
							pass_point = true;
						if (!pass_point)
						{
							VPoint point;
							int point_idx = round(degree * count_num / 360);
							point.timestamp = timestamp - point_idx * (scan_time / count_num);
							point.x = points[i + 3000].range * cos(M_PI / 180 * points[i].degree);
							point.y = -points[i + 3000].range * sin(M_PI / 180 * points[i].degree);
							point.z = 0;
							point.intensity = points[i + 3000].intensity;
							point_cloud->points.push_back(point);
							++point_cloud->width;
						}
					}
					sensor_msgs::PointCloud2 pc_msg;
					pcl::toROSMsg(*point_cloud, pc_msg);
					pointcloud_pub.publish(pc_msg);
				}
			}
			else
			{
				if (pubScan)
				{
					std::vector<ScanPoint> points;
					ros::Time start_time;
					float scan_time;
					this->getScan(points, start_time, scan_time);
					int scan_num =  count_num + 1;

					sensor_msgs::LaserScan msg;
					msg.header.frame_id = frame_id;
					if (use_gps_ts)
					{
						msg.header.stamp = ros::Time(sweep_end_time_gps, sweep_end_time_hardware);
					}
					else
					{
						msg.header.stamp = start_time;
					}
					msg.angle_min = - M_PI;
					msg.angle_max =  M_PI;
					// if (angle_able_max > 360)
					// {
					// 	msg.angle_min = 2 * M_PI * (angle_able_min - 360) / 360;
					// 	msg.angle_max = 2 * M_PI * (angle_able_max - 360) / 360;
					// }
					// else
					// {
					// 	msg.angle_min = 2 * M_PI * angle_able_min / 360;
					// 	msg.angle_max = 2 * M_PI * angle_able_max / 360;
					// }
					msg.angle_increment = 2 * M_PI / (double)(count_num);
					msg.range_min = min_range;
					msg.range_max = max_range;
					msg.ranges.resize(scan_num);
					msg.intensities.resize(scan_num);
					msg.scan_time = scan_time;
					msg.time_increment = scan_time / (double)(count_num);

					for (int k = 0; k < scan_num; k++)
					{
						msg.ranges[k] = std::numeric_limits<float>::infinity();
						msg.intensities[k] = 0;
					}

					int start_num = floor(0 * count_num / 360);
					int end_num = floor(360 * count_num / 360);
					for (int i = 0; i < count_num; i++)
					{
						int point_idx = round((360 - points[i].degree) * count_num / 360);
						if (point_idx < (end_num - count_num))
							point_idx += count_num;
						point_idx = point_idx - start_num;
						if (point_idx < 0 || point_idx >= scan_num)
							continue;
						if (points[i].range == 0.0)
						{
							msg.ranges[point_idx] = std::numeric_limits<float>::infinity();
						}
						else
						{
							double dist = points[i].range;
							msg.ranges[point_idx] = (float)dist;
						}
						msg.intensities[point_idx] = points[i].intensity;
						if(truncated_mode_==1)
							{
								for (int j = 0; j < disable_angle_max_range.size(); ++j) {
									if ((points[i].degree >= (disable_angle_min_range[j]) ) && (points[i].degree <= (disable_angle_max_range[j]))) {
										msg.ranges[point_idx] = std::numeric_limits<float>::infinity();
										msg.intensities[point_idx] = 0;
									}
								}
							}
						msg.intensities[point_idx] = points[i].intensity;
						if(msg.intensities[point_idx] <=5 && msg.intensities[point_idx] >0)
							{
								msg.ranges[point_idx] = std::numeric_limits<float>::infinity();
								msg.intensities[point_idx] = 0;
							}
						if(lidar_name == "M10" || lidar_name == "M10_P" || lidar_name == "M10_PLUS")
							{
								if(msg.intensities[point_idx] != 0)
									{
										msg.ranges[point_idx] = std::numeric_limits<float>::infinity();
										msg.intensities[point_idx] = 0;
									}
							}
					}
					pub_.publish(msg);
				}
				if (pubPointCloud2)
				{
					std::vector<ScanPoint> points;
					ros::Time start_time;
					float scan_time;
					this->getScan(points, start_time, scan_time);
					VPointCloud::Ptr point_cloud(new VPointCloud());
					if (use_gps_ts)
					{
						start_time = ros::Time(sweep_end_time_gps, sweep_end_time_hardware);
					}
					double timestamp = start_time.toSec();
					point_cloud->header.stamp = static_cast<uint64_t>(timestamp * 1e6);
					point_cloud->header.frame_id = frame_id;
					point_cloud->height = 1;
					for (uint16_t i = 0; i < count_num; i++)
					{
						double degree = 360.0 - points[i].degree;
						bool pass_point = false;
						if (angle_able_max < 360)
						{
							if (degree < angle_able_min || degree > angle_able_max)
								pass_point = true;
						}
						else
						{
							if (degree < angle_able_min && degree > (angle_able_max - 360))
								pass_point = true;
						}
						if (points[i].range < 0.001)
							pass_point = true;
						if (!pass_point)
						{
							VPoint point;
							int point_idx = round(degree * count_num / 360);
							point.timestamp = timestamp - point_idx * (scan_time / count_num);
							point.x = points[i].range * cos(M_PI / 180 * points[i].degree);
							point.y = -points[i].range * sin(M_PI / 180 * points[i].degree);
							point.z = 0;
							point.intensity = points[i].intensity;
							point_cloud->points.push_back(point);
							++point_cloud->width;
						}
					}
					sensor_msgs::PointCloud2 pc_msg;
					pcl::toROSMsg(*point_cloud, pc_msg);
					pointcloud_pub.publish(pc_msg);
				}
			}
			count_num = 0;
			wait_for_wake = true;
			if (first_compensation && compensation)
			{
				lidar_difop();
			}
		}
	}

	bool LslidarDriver::polling() // 主循环函数
	{
		if (!is_start)
			return true;
		unsigned char *packet_bytes = new unsigned char[500];
		int len = 0;
		bool difop = false;
		if (interface_selection == "net")
		{

			lslidar_msgs::LslidarPacketPtr packet(
				new lslidar_msgs::LslidarPacket());
			struct timeval tv;
			int last_usec, now_usec;

			while (true)
			{
				difop = false;
				len = 0;
				len = msop_input_->getPacket(packet);
				if (packet->data[0] == 0x5a)
				{
					if (lidar_name == "N10" || lidar_name == "L10")
						len = 58;
					else if (lidar_name == "M10")
						len = 92;
					else if (lidar_name == "N10_P")
						len = 108;
					else if (lidar_name == "M10_GPS")
						len = 102;
					else
					{
						int len_H = packet->data[1];
						int len_L = packet->data[2];
						len = len_H * 256 + len_L;
					}
					for (int i = len - 1; i > 0; i--)
						packet->data[i] = packet->data[i - 1];
					packet->data[0] = 0xa5;
				}

				if (lidar_name == "N10" || lidar_name == "L10")
					len = 58;
				else if (lidar_name == "M10")
					len = 92;
				else if (lidar_name == "N10_P")
					len = 108;
				else if (lidar_name == "M10_GPS")
					len = 102;
				else
				{
					int len_H = packet->data[2];
					int len_L = packet->data[3];
					len = len_H * 256 + len_L;
				}
				if ((lidar_name == "M10" || lidar_name == "M10_DOUBLE" || lidar_name == "M10_GPS" || lidar_name == "M10_P" || lidar_name == "M10_PLUS") && compensation)
				{
					if (packet->data[2] == 0x55 && packet->data[3] == 0x00 && packet->data[186] == 0xFA && packet->data[187] == 0xFB)
					{
						len = 188;
						difop = true;
					}
				}

				if (len <= 0 || len >= 1000 || packet->data[0] != 0xa5 || packet->data[1] != 0x5a)
					continue;
				for (int i = 0; i < len; i++)
				{
					packet_bytes[i] = packet->data[i];
				}
				if ((lidar_name == "N10" || lidar_name == "L10" || lidar_name == "N10_P") && packet_bytes[len - 1] != N10_CalCRC8(packet_bytes, len - 1))
					continue;
				break;
			}
		}
		else
		{
			if (in_file_name != "") // 读txt文件功能
			{
				int usleep_time = round(1000000 / 10 / 24) - 135;
				while (true)
				{
					std::ifstream file_reader(in_file_name);
					while (file_reader.peek() != EOF)
					{
						std::string line;
						std::getline(file_reader, line, '\n');
						for (int i = 0; i < line.size() - 1; i++)
						{
							line[i] = line[i] - 48;
							if (line[i] > 9)
								line[i] = line[i] - 39;
						}

						for (int i = 0; i < (line.size() - 1) / 2; i++)
						{
							packet_bytes[i] = line[i * 2] * 16 + line[i * 2 + 1];
						}
						if (lidar_name == "N10" || lidar_name == "L10")
							len = 58;
						else if (lidar_name == "M10")
							len = 92;
						else if (lidar_name == "N10_P")
							len = 108;
						else if (lidar_name == "M10_GPS")
							len = 102;
						else
						{
							int len_H = packet_bytes[2];
							int len_L = packet_bytes[3];
							len = len_H * 256 + len_L;
						}
						// if (len >= 140 )
						// 	continue;
						if (lidar_name == "N10_P" || lidar_name == "M10_DOUBLE")
							LslidarDriver::data_processing_2(packet_bytes, len);
						else
							LslidarDriver::data_processing(packet_bytes, len);
						usleep(usleep_time);
					}
				}
				return false;
			}
			else
			{
				while (true)
				{
					difop = false;
					len = 0;
					len = LslidarDriver::receive_data(packet_bytes);
					if ((lidar_name == "M10" || lidar_name == "M10_DOUBLE" || lidar_name == "M10_GPS" || lidar_name == "M10_P" || lidar_name == "M10_PLUS") && compensation)
					{
						if (packet_bytes[2] == 0x55 && packet_bytes[3] == 0x00 && packet_bytes[186] == 0xFA && packet_bytes[187] == 0xFB)
							difop = true;
					}
					//printf("len = %d\n",len);
					if (len == 0)
						continue;
					break;
				}
			}
		}
		if (difop)
			LslidarDriver::difop_processing(packet_bytes);
		else
		{
			if (lidar_name == "N10_P" || lidar_name == "M10_DOUBLE")
				LslidarDriver::data_processing_2(packet_bytes, len);
			else
				LslidarDriver::data_processing(packet_bytes, len);
		}
		delete packet_bytes;
		return true;
	}

} // namespace lslidar_driver
