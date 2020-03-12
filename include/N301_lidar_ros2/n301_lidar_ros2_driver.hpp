#ifndef LSLIDAR_N301_DRIVER_H
#define LSLIDAR_N301_DRIVER_H

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <string>

#include <boost/shared_ptr.hpp>

#include <rclcpp/rclcpp.hpp>
#include <N301_lidar_ros2/LslidarN301Packer.h>

namespace lslidar_n301_msgs = N301_lidar_ros2;

namespace lslidar_n301_driver {

static uint16_t PACKET_SIZE = 1206;

class LslidarDriver: public rclcpp::Node {
public:
    explicit LslidarDriver(const std::string& frame_id = "lslidar",
                            const std::string& device_ip_string = "192.168.1.222",
                            const int& device_port = 2368);
    LslidarDriver(const LslidarDriver&) = delete;
    LslidarDriver& operator =(const LslidarDriver&) = delete;
    ~LslidarDriver() { (void) close(socket_id); }

    bool initialize();
    bool polling();

    typedef boost::shared_ptr<LslidarDriver> Ptr;
    typedef boost::shared_ptr<const LslidarDriver> ConstPtr;

private:
    bool loadParameters();
    bool createRosIO();
    bool openUDPPort();
    int getPacket(lslidar_n301_msgs::LslidarN301PacketPtr& msg);

    // Ethernet relate variables
    std::string device_ip_string;
    in_addr device_ip;
    int UDP_PORT_NUMBER;
    int socket_id;

    // ROS related variables
    std::string frame_id;
    rclcpp::Publisher packet_pub;
    rclcpp::Parameter p_frame_id, p_device_ip, p_device_port;

};

}   // namespace lslidar_n301_driver

#endif  // LSLIDAR_N301_DRIVER_H