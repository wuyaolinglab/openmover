#include "lslidar_driver/input.h"

extern volatile sig_atomic_t flag;
namespace lslidar_driver
{
    static const size_t packet_size_input = 400;
    ////////////////////////////////////////////////////////////////////////
    // Input base class implementation
    ////////////////////////////////////////////////////////////////////////

    /** @brief constructor
     *
     *  @param private_nh ROS private handle for calling node.
     *  @param port UDP port number.
     */
    Input::Input(rclcpp::Node *private_nh, uint16_t port) : private_nh_(private_nh), port_(port) {
        npkt_update_flag_ = false;
        cur_rpm_ = 0;
        return_mode_ = 1;
        devip_str_difop = std::string("192.168.1.200");
        devip_str_ = std::string("192.168.1.102");
        lidar_name = std::string("M10");
        add_multicast = false;
        group_ip = std::string("224.1.1.2");
        UDP_PORT_NUMBER_DIFOP = 2369;


	    private_nh->declare_parameter<std::string>("device_ip","192.168.1.102");
        private_nh->declare_parameter<std::string>("device_ip_difop","192.168.1.200");
        private_nh->declare_parameter<bool>("add_multicast",false);
        private_nh->declare_parameter<std::string>("group_ip","224.1.1.2");
        private_nh->declare_parameter<int>("difop_port",2369);


        private_nh->get_parameter("lidar_name", lidar_name);
        private_nh->get_parameter("device_ip", devip_str_);
        private_nh->get_parameter("add_multicast", add_multicast);
        private_nh->get_parameter("group_ip", group_ip);
        private_nh->get_parameter("difop_port", UDP_PORT_NUMBER_DIFOP);
        private_nh->get_parameter("device_ip_difop", devip_str_difop);

        if (!devip_str_.empty())
            RCLCPP_INFO(private_nh->get_logger(), "[driver][input] accepting packets from IP address: %s  port: %d",
                        devip_str_.c_str(),port);
    }

    /** @brief constructor
     *
     *  @param private_nh ROS private handle for calling node.
     *  @param port UDP port number
     */
    InputSocket::InputSocket(rclcpp::Node *private_nh, uint16_t port) : Input(private_nh, port) {
        sockfd_ = -1;

        if (!devip_str_.empty()) {
            inet_aton(devip_str_.c_str(), &devip_);
            inet_aton(devip_str_difop.c_str(), &devip_difop);
        }

        RCLCPP_INFO(private_nh_->get_logger(), "[driver][socket] Opening UDP socket: port %d", port);
        sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
        if (sockfd_ == -1) {
            perror("socket");  // TODO: ROS_ERROR errno
            return;
        }

        int opt = 1;
        if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (const void *) &opt, sizeof(opt))) {
            perror("setsockopt error!\n");
            return;
        }

        sockaddr_in my_addr;                   // my address information
        memset(&my_addr, 0, sizeof(my_addr));  // initialize to zeros
        my_addr.sin_family = AF_INET;          // host byte order
        my_addr.sin_port = htons(port);        // port in network byte order
        my_addr.sin_addr.s_addr = INADDR_ANY;  // automatically fill in my IP

        if (bind(sockfd_, (sockaddr * ) & my_addr, sizeof(sockaddr)) == -1) {
            perror("bind");  // TODO: ROS_ERROR errno
            return;
        }

        if (add_multicast) {
            struct ip_mreq group;
            group.imr_multiaddr.s_addr = inet_addr(group_ip.c_str());
            group.imr_interface.s_addr = htonl(INADDR_ANY);

            if (setsockopt(sockfd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *) &group, sizeof(group)) < 0) {
                perror("Adding multicast group error ");
                close(sockfd_);
                exit(1);
            } else
                printf("Adding multicast group...OK.\n");
        }
        if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
            perror("non-block");
            return;
        }
    }

    /** @brief destructor */
    InputSocket::~InputSocket(void) {
        (void) close(sockfd_);
    }

    void Input::UDP_difop()
    {
        sockaddr_in server_sai;
        server_sai.sin_family = AF_INET; // IPV4 协议族
        server_sai.sin_port = htons(UDP_PORT_NUMBER_DIFOP);
        server_sai.sin_addr.s_addr = inet_addr(devip_str_.c_str());
        for (int k = 0; k < 10; k++)
        {
            unsigned char data[188]= {0x00};
            data[0] = 0xA5;
            data[1] = 0x5A;
            data[2] = 0x55;
            data[184] = 0x08;
            data[185] = 0x01;
            data[186] = 0xFA;
            data[187] = 0xFB;
            int rtn = sendto(sockfd_, data, 188, 0, (struct sockaddr *)&server_sai, sizeof(struct sockaddr));
            if (rtn < 0)    printf("start scan error !\n");
            else return; 
        }
        return; 
    }
    
    void Input::UDP_order(const std_msgs::msg::Int8 msg)
    {
        int i = msg.data;
        sockaddr_in server_sai;
        server_sai.sin_family = AF_INET; // IPV4 协议族
        server_sai.sin_port = htons(UDP_PORT_NUMBER_DIFOP);
        server_sai.sin_addr.s_addr = inet_addr(devip_str_.c_str());
        int rtn = 0;
        for (int k = 0; k < 10; k++)
        {
            unsigned char data[188]= {0x00};
            data[0] = 0xA5;
            data[1] = 0x5A;
            data[2] = 0x55;
            data[186] = 0xFA;
            data[187] = 0xFB;  
            if(lidar_name == "M10" || lidar_name == "M10_GPS" || lidar_name == "M10_P"){
                if (i <= 1){				    //雷达启停
                    data[184] = 0x01;
                    data[185] = char(i);
                }
                else if (i == 2){			    //雷达点云不滤波
                    data[181] = 0x0A;
                    data[184] = 0x06;
                    data[185] = 0x01;
                }
                else if (i == 3){				//雷达点云正常滤波
                    data[181] = 0x0B;
                    data[184] = 0x06;
                    data[185] = 0x01;
                }
                else if (i == 4){				//雷达近距离滤波
                    data[181] = 0x0C;
                    data[184] = 0x06;
                    data[185] = 0x01;
                }    
                else if (i == 100){				//接收设备包
                    data[184] = 0x08;
                    data[185] = 0x01;
                }      
                else return;    
            }
            else if (lidar_name == "M10_PLUS"){   
                data[184] = 0x0A;
                data[185] = 0x01;           
                if(i == 5)       {
                data[141] = 0x01;
                data[142] = 0x2c;
                }
                else if(i == 6)  {
                data[141] = 0x01;
                data[142] = 0x68;
                }
                else if(i == 8)  {
                data[141] = 0x01;
                data[142] = 0xe0;
                }
                else if(i == 10) {
                data[141] = 0x02;
                data[142] = 0x58;
                }
                else if(i == 12) {
                data[141] = 0x02;
                data[142] = 0xd0;
                }
                else if(i == 15) {
                data[141] = 0x03;
                data[142] = 0x84;
                }
                else if(i == 20) {
                data[141] = 0x04;
                data[142] = 0xb0;
                }
                else if(i <= 1)  {
                data[184] = 0x01;
                data[185] = char(i);
                }
                else if(i == 100) {				//接收设备包
                    data[184] = 0x08;
                    data[185] = 0x01;
                }      
                else return;
            }
            else if(lidar_name == "N10"){
                if(i <= 1){
                data[185] = char(i);
                data[184] = 0x01;
                }
                else if(i>=6 && i<=12){
                data[172] = char(i);
                data[184] = 0x0a;
                data[185] = 0X01;
                }
                else return; 
            }
            rtn = sendto(sockfd_, data, 188, 0, (struct sockaddr *)&server_sai, sizeof(struct sockaddr));
            if (rtn < 0)
            {
                printf("start scan error !\n");
            }
            else
            {
                if (i == 1)
                    usleep(3000000);
                return;
            }
        }
        return;
    }



    int InputSocket::getPacket(lslidar_msgs::msg::LslidarPacket::UniquePtr &packet)
    {
        int q = 0;
        struct pollfd fds[1];
        fds[0].fd = sockfd_;
        fds[0].events = POLLIN;
        static const int POLL_TIMEOUT = 2000; // one second (in msec)

        sockaddr_in sender_address{};
        socklen_t sender_address_len = sizeof(sender_address);
        while (flag == 1)
        {
            // poll() until input available
            do {
                int retval = poll(fds, 1, POLL_TIMEOUT);
                if (retval < 0)  // poll() error?
                {
                    if (errno != EINTR)
                        RCLCPP_ERROR(private_nh_->get_logger(), "[driver][socket] poll() error: %s", strerror(errno));
                    return 0;
                }
                if (retval == 0) // poll() timeout?
                {
                    RCLCPP_WARN(private_nh_->get_logger(), "lslidar poll() timeout, port: %d",port_);
                    return 0;
                }
                if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL)) // device error?
                {
                    RCLCPP_ERROR(private_nh_->get_logger(),"poll() reports lslidar error");
                    return 0;
                }
            } while ((fds[0].revents & POLLIN) == 0);

            // Receive packets that should now be available from the
            // socket using a blocking read.
            ssize_t nbytes = recvfrom(sockfd_, &packet->data[0], packet_size_input, 0,
                                      (sockaddr *)&sender_address, &sender_address_len);
            //        ROS_DEBUG_STREAM("incomplete lslidar packet read: "
            //                         << nbytes << " bytes");
            q = (int)nbytes;
            if (nbytes < 0)
            {
                if (errno != EWOULDBLOCK)
                {
                    perror("recvfail");
                    RCLCPP_ERROR(private_nh_->get_logger(),"recvfail");
                    return 1;
                }
            }
            else if ((size_t)nbytes <= packet_size_input || (size_t)nbytes >= 50)
            {

                // read successful,
                // if packet is not from the lidar scanner we selected by IP,
                // continue otherwise we are done
                if (devip_str_ != "" && sender_address.sin_addr.s_addr != devip_.s_addr)
                    continue;
                else
                    break; // done
            }
            
        }
        if (flag == 0)
        {
            abort();
        }

        return q;
    }
    InputPCAP::InputPCAP(rclcpp::Node *private_nh, uint16_t port, double packet_rate, std::string filename) : Input(private_nh, port),
                                                                                packet_rate_(packet_rate),
                                                                                filename_(filename)
    {
        pcap_ = NULL;
        empty_ = true;
        read_once_ = false;
        read_fast_ = false;
        repeat_delay_ = 0.0;
        private_nh->get_parameter("read_once", read_once_);
        private_nh->get_parameter("read_fast", read_fast_);
        private_nh->get_parameter("repeat_delay", repeat_delay_);

        if (read_once_)
            RCLCPP_WARN(private_nh_->get_logger(),"Read input file only once.");
        if (read_fast_)
            RCLCPP_WARN(private_nh_->get_logger(),"Read input file as quickly as possible.");
        if (repeat_delay_ > 0.0)
            RCLCPP_WARN(private_nh_->get_logger(),"Delay %.3f seconds before repeating input file.", repeat_delay_);

        RCLCPP_INFO(private_nh_->get_logger(),"Opening PCAP file %s",filename_.c_str());
        if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_)) == NULL)
        {
            RCLCPP_WARN(private_nh_->get_logger(),"Error opening lslidar socket dump file.");
            return;
        }
        std::stringstream filter;
        if (devip_str_ != "")
        {
            filter << "src host " << devip_str_ << "&&";
        }
        filter << "udp dst port " << port;
        pcap_compile(pcap_, &pcap_packet_filter_, filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);
    }

    InputPCAP::~InputPCAP(void)
    {
        pcap_close(pcap_);
    }

    int InputPCAP::getPacket(lslidar_msgs::msg::LslidarPacket::UniquePtr &pkt)
    {
        struct pcap_pkthdr *header;
        const u_char *pkt_data;
        while (flag == 1)
            {
            int res;
            if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0)
            {
                // skip packets not for the correct port and from the selected IP address
                if (!devip_str_.empty() && (0 == pcap_offline_filter(&pcap_packet_filter_, header, pkt_data)))
                    continue;

                if (read_fast_ == false)
                    packet_rate_.sleep();
                mempcpy(&pkt->data[0], pkt_data + 42, packet_size_input);
                empty_ = false;
                return 0;
            }
            if (empty_)
            {
                RCLCPP_WARN(private_nh_->get_logger(),"Error %d reading lslidar packet: %s", res, pcap_geterr(pcap_));
                return -1;
            }
            if (read_once_)
            {
                RCLCPP_WARN(private_nh_->get_logger(),"end of file reached -- done reading.");
                return -1;
            }
            if (repeat_delay_ > 0.0)
            {
                RCLCPP_WARN(private_nh_->get_logger(),"end of file reached -- delaying %.3f seconds.", repeat_delay_);
                usleep(rint(repeat_delay_ * 1000000.0));
            }
            RCLCPP_WARN(private_nh_->get_logger(),"replayding lsliar dump file");

            pcap_close(pcap_);
            pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
            empty_ = true;
        }
        if (flag == 0)
        {
            abort();
        }
        return 0;
    }

} // namespace
