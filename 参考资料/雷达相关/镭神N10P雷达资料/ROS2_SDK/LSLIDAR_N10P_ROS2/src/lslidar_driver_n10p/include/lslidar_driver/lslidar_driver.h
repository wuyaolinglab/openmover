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

#ifndef LSLIDAR_DRIVER_H
#define LSLIDAR_DRIVER_H

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <string>

#include <boost/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include "rclcpp/rclcpp.hpp"
#include <thread>
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"
#include "lslidar_msgs/msg/lslidar_packet.hpp"
#include "std_msgs/msg/byte.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"

#include "time.h"
#include "input.h"
#include "lsiosr.h"
#include "sensor_msgs/msg/laser_scan.hpp"
namespace lslidar_driver {

struct PointXYZIT {
    PCL_ADD_POINT4D;
    uint8_t intensity;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;
 
typedef struct {
    double degree;
    double range;
    double intensity;
} ScanPoint;

class LslidarDriver: public rclcpp::Node {
public:
	LslidarDriver();
	LslidarDriver(const rclcpp::NodeOptions& options);
	~LslidarDriver();

    bool initialize();
    bool polling();

    typedef std::shared_ptr<LslidarDriver> LslidarDriverPtr;
    typedef std::shared_ptr<const LslidarDriver> LslidarDriverConstPtr;

private:    
    uint64_t get_gps_stamp(struct tm t);
    uint8_t N10_CalCRC8(unsigned char * p, int len);
    bool loadParameters();
    bool createRosIO();
    void open_serial();
    void lidar_difop();
    void lidar_order(const std_msgs::msg::Int8::SharedPtr msg);
    void data_processing(unsigned char *packet_bytes,int len);
    void data_processing_2(unsigned char *packet_bytes,int len);
    void difop_processing(unsigned char *packet_bytes);
    void pubScanThread();
    void recvThread_crc(int &count,int &link_time);
    int receive_data(unsigned char *packet_bytes);
    int getScan(std::vector<ScanPoint> &points, rclcpp::Time &scan_time, float &scan_duration);

    boost::thread *pubscan_thread_ ;
    boost::shared_ptr<Input> msop_input_;
    boost::mutex mutex_; 
    boost::mutex pubscan_mutex_;
    boost::condition_variable pubscan_cond_;
    
    int UDP_PORT_NUMBER; 
    int count_num;
    int package_points;
    int data_bits_start;
    int degree_bits_start;
    int end_degree_bits_start;
    int rpm_bits_start;
	int baud_rate_;
    int points_size_;
    int idx = 0;
    int link_time = 0;

    bool use_gps_ts;   
    bool is_start;
    bool high_reflection;
    bool compensation;
    bool first_compensation = true;
    bool pubScan;
    bool pubPointCloud2;

    double min_range;
    double max_range;
    double angle_disable_min;
    double angle_disable_max;
    double angle_able_min;
    double angle_able_max;
    double last_degree = 0.0;	
    double degree_compensation = 0.0;

    uint16_t PACKET_SIZE ;
    uint64_t sweep_end_time_gps;
    uint64_t sweep_end_time_hardware;
    uint64_t sub_second;

    std::string frame_id;
    std::string interface_selection;
    std::string scan_topic;
    std::string lidar_name;
    std::string serial_port_;
    std::string dump_file;
    std::string pointcloud_topic;
    std::string in_file_name;
    
    tm pTime;    
    rclcpp::Time pre_time_;
    rclcpp::Time time_;
    std::vector<ScanPoint> scan_points_;
    std::vector<ScanPoint> scan_points_bak_;
    // Diagnostics updater
    diagnostic_updater::Updater diagnostics;
    std::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic;
    double diag_min_freq;
    double diag_max_freq;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub;
	rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr difop_switch;
    LSIOSR * serial_;
};
typedef PointXYZIT VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

} // namespace lslidar_driver
POINT_CLOUD_REGISTER_POINT_STRUCT(lslidar_driver::PointXYZIT,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                          std::uint8_t, intensity,
                                          intensity)(double, timestamp, timestamp))
#endif // _LSLIDAR_DRIVER_H_
