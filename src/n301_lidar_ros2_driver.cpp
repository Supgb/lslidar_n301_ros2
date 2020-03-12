#include <string>
#include <cmath>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <chrono>
#include <memory>

#include <tf2_ros/transform_listener.h>
#include <N301_lidar_ros2/n301_lidar_ros2_driver.hpp>

namespace lslidar_n301_driver {

LslidarDriver::LslidarDriver(const std::string& frame_id = "lslidar",
                            const std::string& device_ip_string = "192.168.1.222",
                            const int& device_port = 2368):
    device_ip_string(device_ip_string),
    device_ip(),
    UDP_PORT_NUMBER(device_port),
    socket_id(-1),
    frame_id(frame_id),
    packet_pub(),
    p_frame_id("frame_id", frame_id),
    p_device_ip("device_ip", device_ip_string),
    p_device_port("device_port", UDP_PORT_NUMBER) {}

bool LslidarDriver::loadParameters() {
    this->set_parameter(p_frame_id);
    this->set_parameter(p_device_ip);
    this->set_parameter(p_device_port);

    inet_aton(device_ip_string.c_str(), &device_ip);
    RCLCPP_INFO_STREAM(this->get_logger(), "Opening UDP socket: address " << device_ip_string);
    RCLCPP_INFO_STREAM(this->get_logger(), "Opening UDP socket: port " << UDP_PORT_NUMBER);
    
    return true;
}

bool LslidarDriver::createRosIO() {
    // TODO: diagnostics confirgurations.

    packet_pub = this->create_publisher<lslidar_n301_msgs::LslidarN301Packet>(
        "lslidar_packet", 100
    )
    return true;
}

bool LslidarDriver::openUDPPort() {
    socket_id = socket(PF_INET, SOCK_DGRAM, 0);
    if (socket_id == -1) {
        perror("socket");
        return false;
    }
    sockaddr_in host_addr;
    memset(&host_addr, 0, sizeof(host_addr));
    host_addr.sin_family = AF_INET;
    host_addr.sin_port = htons(UDP_PORT_NUMBER);
    RCLCPP_INFO_STREAM(this->get_logger(), "Opening UDP socket: port " << UDP_PORT_NUMBER);
    host_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(socket_id, reinterpret_cast<sockaddr *>(&my_addr), sizeof(sockaddr)) == -1) {
        perror("bind");
        return false;
    }
    
    if (fcntl(socket_id, F_SETFL, O_NONBLOCK|FASYNC) < 0) { 
        perror("non-block");
        return false;
    }

    return true;
}

bool LslidarN301Driver::initialize() {
    if (!loadParameters()) {
        RCLCPP_ERROR("Could not load all required ROS2 parameters...");
        return false;
    }
    if (!createRosIO()) {
        RCLCPP_ERROR("Could not create all ROS2 IO...");
        return false;
    }
    if (!openUDPPort()) {
        RCLCPP_ERROR("Could not open UDP port...");
        return false;
    }
    RCLCPP_INFO("Initialized lslidar without error");
    return true;
}

int LslidarDriver::getPacket(lslidar_n301_msgs::LslidarN301PacketPtr& packet) {
    rclcpp::Time time1 = this->now();
    struct pollfd fds[1];
    fds[0].fd = socket_id;
    fds[0].events = POLLIN;

    static const int POLL_TIMEOUT = 2000;

    sockaddr_in sender_address;
    socklen_t sender_address_len = sizeof(sender_address);

    while (1) {
        do {
            int retval = poll(fds, 1, POLL_TIMEOUT);
            if (retval < 0) {   // error?
                if (errno != EINTR)
                    RCLCPP_ERROR(this->get_logger(), strerror(errno));
                return 1;
            }
            if (retval == 0) {  // timeout?
                RCLCPP_WARN("lslidar poll() timeout...");
                return 1;
            }
            if ((fds[0].revents & POLLERR)
                || (fds[0].revents & POLLHUP)
                || (fds[0].revents & POLLNVAL)) {   // device error? 
                    RCLCPP_ERROR("poll() reports lslidar error");
                    return 1;
                }
        } while ((fds[0].revents & POLLERR) == 0);  // poll until input available

        ssize_t nbytes = recvfrom(socket_id, &packet->data[0], PACKET_SIZE, 0
            reinterpret_cast<sockaddr*>(&sender_address), &sender_address_len);

        if (nbytes < 0) {
            if (errno != EWOULDBLOCK) {
                perror("recvfail");
                RCLCPP_ERROR("recvfail");
                return 1;
            }
        } else if (static_cast<size_t>(nbytes) == PACKET_SIZE) {    // Success.
            if (sender_address.sin_addr.s_addr != device_ip.s_addr) 
                continue;
            break;
        }
    }   // end-while
    packet->stamp = time1 + (this->now() - time1);
    return 0;
}

bool LslidarDriver::polling() {
    // For zero-copy sharing.
    std::unique_ptr packet =  std::make_unique<lslidar_n301_msgs::LslidarN301Packet>();
    while (1) {
        int rc = getPacket(packet);
        if (rc == 0) break; // got a packet?
        if (rc < 0) return false;
    }
    RCLCPP_DEBUG("Publishing a full lslidar scan.").
    packet_pub.publish(*packet);

    // TODO: diagnostics issuses.

    return true;
}

}   // namespace lslidar_n301_driver