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
#include "input.h"

#include <boost/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include "lsiosr.h"
#include <sensor_msgs/LaserScan.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <lslidar_msgs/LslidarPacket.h>
#include <std_msgs/Byte.h>
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

uint16_t PACKET_SIZE ;

class LslidarDriver {
public:

    LslidarDriver(ros::NodeHandle& n, ros::NodeHandle& pn);
    ~LslidarDriver();

    bool initialize();
    bool polling();
    int getScan(std::vector<ScanPoint> &points, ros::Time &scan_time, float &scan_duration);
    void data_processing(unsigned char *packet_bytes,int len);
    void data_processing_2(unsigned char *packet_bytes,int len);
    void difop_processing(unsigned char *packet_bytes);
    typedef boost::shared_ptr<LslidarDriver> LslidarDriverPtr;
    typedef boost::shared_ptr<const LslidarDriver> LslidarDriverConstPtr;
    void recvThread_crc(int &count_2,int &link_time);
private:
    boost::mutex mutex_; 
    boost::mutex pubscan_mutex_;
    boost::condition_variable pubscan_cond_;

    boost::thread *recv_thread_;
    boost::thread *pubscan_thread_ ;
    bool createRosIO();
    void close_serial();
    void open_serial();
    void pubScanThread();
    void lidar_difop();
    void lidar_order(const std_msgs::Int8 msg);
    int receive_data(unsigned char *packet_bytes);
    void initParam();
    uint8_t N10_CalCRC8(unsigned char * p, int len);
    // Ethernet relate variables
    int UDP_PORT_NUMBER;
	bool is_start;
    // ROS related variables
    LSIOSR * serial_;
    std::string serial_port_;
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    std::string  interface_selection;
    boost::shared_ptr<Input> msop_input_;
    ros::Publisher packet_pub;
	ros::Publisher pointcloud_pub;

	ros::Subscriber difop_switch;
    // Diagnostics updater
    diagnostic_updater::Updater diagnostics;
    boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic;
    double diag_min_freq;
    double diag_max_freq;

    std::vector<ScanPoint> scan_points_;
    std::vector<ScanPoint> scan_points_bak_;
    std::string frame_id;
    std::string lidar_name;
    std::string scan_topic;
    std::string dump_file;
    std::string pointcloud_topic;
    std::string in_file_name;
    double min_range;
    double max_range;
    double angle_disable_min;
    double angle_disable_max;
    double angle_able_min;
    double angle_able_max;
    double degree_compensation = 0.0;
    bool use_gps_ts;
    bool high_reflection;
    bool n10p_double_echo;
    bool compensation;
    bool first_compensation = true;
    bool pubScan;
    bool pubPointCloud2;
    int count_num;
    int package_points;
    int data_bits_start;
    int degree_bits_start;
    int end_degree_bits_start;
    int rpm_bits_start;
	int baud_rate_;
    int points_size_;
    ros::Time pre_time_;
    ros::Time time_;
    ros::Publisher pub_;
    tm pTime;    
    uint64_t sub_second;
    uint64_t get_gps_stamp(tm t);
    uint64_t sweep_end_time_gps;
    uint64_t sweep_end_time_hardware;
    int idx = 0;
    int link_time = 0;
    double last_degree = 0.0;	

    int truncated_mode_;
    double min_distance,max_distance;
    std::vector<int> disable_angle_min_range, disable_angle_max_range, disable_angle_range_default;
};

typedef LslidarDriver::LslidarDriverPtr LslidarDriverPtr;
typedef LslidarDriver::LslidarDriverConstPtr LslidarDriverConstPtr;
typedef PointXYZIT VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;
} // namespace lslidar_driver
POINT_CLOUD_REGISTER_POINT_STRUCT(lslidar_driver::PointXYZIT,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                          std::uint8_t, intensity,
                                          intensity)(double, timestamp, timestamp))
#endif // _LSLIDAR__DRIVER_H_
