#ifndef LSLIDAR_N301_DECODER_H
#define LSLIDAR_N301_DECODER_H

#include <cmath>
#include <vector>
#include <string>
//#include "boost/shared_ptr.hpp"
#include <memory>
#include "time.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"

#include "n301_lidar_ros2/msg/lslidar_n301_packet.hpp"
#include "n301_lidar_ros2/msg/lslidar_n301_point.hpp"
#include "n301_lidar_ros2/msg/lslidar_n301_scan.hpp"
#include "n301_lidar_ros2/msg/lslidar_n301_sweep.hpp"
#include "std_msgs/msg/header.hpp"

namespace lslidar_n301_decoder {

const double DEG_TO_RAD = 0.017453292;
const double RAD_TO_DEG = 57.29577951;

static const int SIZE_BLOCK = 100;
static const int RAW_SCAN_SIZE = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

static const double DISTANCE_MAX = 130.0;   // meters.
static const double DISTANCE_RESOLUTION = 0.002;    // meters.
static const double DISTANCE_MAX_UNITS = (DISTANCE_MAX / DISTANCE_RESOLUTION + 1.0);

static const uint16_t UPPER_BANK = 0xeeff;
static const uint16_t LOWER_BANK = 0xddff;

static const int FIRINGS_PER_BLOCK = 2;
static const int SCANS_PER_FIRING = 16;
static const double BLOCK_TDURATION = 110.592;  // us
static const double DSR_TOFFSET = 2.304;        // us
static const double FIRING_TOFFSET = 55.296;    // us

static const int PACKET_SIZE        = 1206;
static const int BLOCKS_PER_PACKET  = 12;
static const int PACKET_STATUS_SIZE = 4;
static const int SCANS_PER_PACKET =
        (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);
static const int FIRINGS_PER_PACKET =
        FIRINGS_PER_BLOCK * BLOCKS_PER_PACKET;

// Pre-compute the sine and cosine for the altitude angles.
static const double scan_altitude[16] = {
    -0.2617993877991494,   0.017453292519943295,
    -0.22689280275926285,  0.05235987755982989,
    -0.19198621771937624,  0.08726646259971647,
    -0.15707963267948966,  0.12217304763960307,
    -0.12217304763960307,  0.15707963267948966,
    -0.08726646259971647,  0.19198621771937624,
    -0.05235987755982989,  0.22689280275926285,
    -0.017453292519943295, 0.2617993877991494
};

static const double cos_scan_altitude[16] = {
    std::cos(scan_altitude[ 0]), std::cos(scan_altitude[ 1]),
    std::cos(scan_altitude[ 2]), std::cos(scan_altitude[ 3]),
    std::cos(scan_altitude[ 4]), std::cos(scan_altitude[ 5]),
    std::cos(scan_altitude[ 6]), std::cos(scan_altitude[ 7]),
    std::cos(scan_altitude[ 8]), std::cos(scan_altitude[ 9]),
    std::cos(scan_altitude[10]), std::cos(scan_altitude[11]),
    std::cos(scan_altitude[12]), std::cos(scan_altitude[13]),
    std::cos(scan_altitude[14]), std::cos(scan_altitude[15]),
};

static const double sin_scan_altitude[16] = {
    std::sin(scan_altitude[ 0]), std::sin(scan_altitude[ 1]),
    std::sin(scan_altitude[ 2]), std::sin(scan_altitude[ 3]),
    std::sin(scan_altitude[ 4]), std::sin(scan_altitude[ 5]),
    std::sin(scan_altitude[ 6]), std::sin(scan_altitude[ 7]),
    std::sin(scan_altitude[ 8]), std::sin(scan_altitude[ 9]),
    std::sin(scan_altitude[10]), std::sin(scan_altitude[11]),
    std::sin(scan_altitude[12]), std::sin(scan_altitude[13]),
    std::sin(scan_altitude[14]), std::sin(scan_altitude[15]),
};

typedef struct{
    double distance;
    double intensity;
}point_struct;

struct PointXYZIT {
    PCL_ADD_POINT4D;
    uint8_t intensity;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;

typedef PointXYZIT VPoint;
typedef pcl::PointCloud<PointXYZIT> VPointCloud;

// Decoder definition.
class LslidarDecoder : public rclcpp::Node {
public:
    enum FLAG { 
        NONE=0,
        PUBLISH_PCL,  // 0x01
        USE_GPS_TS  // 0x02
    };

    explicit LslidarDecoder (const std::string&, const int& = 1000, const double& = 0.5,
                            const double& = 100.0, const double& = -1,
                            const double& = -1, const double& = 20.0,
                            const std::string& = "map", const std::string& = "lslidar",
                            const FLAG& = NONE);    // 0x01
    LslidarDecoder(const LslidarDecoder&) = delete;
    LslidarDecoder& operator =(const LslidarDecoder&) = delete;
    ~LslidarDecoder() = default;

    bool initialize();

    typedef std::shared_ptr<LslidarDecoder> Ptr;
    typedef std::shared_ptr<const LslidarDecoder> ConstPtr;

private:
    union TwoBytes {
        uint16_t distance;
        uint8_t  bytes[2];
    };

    struct RawBlock {
        uint16_t header;        ///< UPPER_BANK or LOWER_BANK
        uint16_t rotation;      ///< 0-35999, divide by 100 to get degrees
        uint8_t  data[BLOCK_DATA_SIZE];
    };

    struct RawPacket {
        RawBlock blocks[BLOCKS_PER_PACKET];
        // uint32_t time_stamp;
        uint8_t time_stamp_yt[4];
        uint8_t factory[2];
        //uint16_t revolution;
        //uint8_t status[PACKET_STATUS_SIZE];
    };


    struct Firing {
        // Azimuth associated with the first shot within this firing.
        double firing_azimuth;
        double azimuth[SCANS_PER_FIRING];
        double distance[SCANS_PER_FIRING];
        double intensity[SCANS_PER_FIRING];
    };

    bool loadParameters();
    bool createRosIO();

    // Callback function for a single lslidar packet.
    bool checkPacketValidity(const RawPacket* packet);
    void decodePacket(const RawPacket* packet);
    void packetCallback(n301_lidar_ros2::msg::LslidarN301Packet::UniquePtr msg);
    // Publish
    //void publishPointCloud();
    void publishScan();
    
    // Check if a point is in the required range.
    bool isPointInRange(const double& distance) {
        return (distance >= min_range && distance <= max_range);
    }

    double rawAzimuthToDouble(const uint16_t& raw_azimuth) {
        return static_cast<double>(raw_azimuth) / 100.0 * DEG_TO_RAD;
    }

    // Calc the means point
    point_struct getMeans(std::vector<point_struct> clusters);

    // Get GPS_base time
    uint64_t get_gps_stamp(tm t);
    
    tm pTime;
    uint64_t packet_timestamp;
    uint64_t sweep_end_time_gps;
    uint64_t sweep_end_time_hardware;

    // Configuration degree base
    int point_num;
    double angle_base;

    // Configuration parameters
    double min_range;
    double max_range;
    double angle_disable_min;
    double angle_disable_max;
    double frequency;
    bool publish_point_cloud;
    bool use_gps_ts;

    double cos_azimuth_table[6300];
    double sin_azimuth_table[6300];

    bool is_first_sweep;
    double last_azimuth;
    double sweep_start_time;
    double packet_start_time;
    Firing firings[FIRINGS_PER_PACKET];

    // ROS related parameters
    std::string fixed_frame_id;
    std::string child_frame_id;
    rclcpp::Parameter p_point_num, p_min_range, p_max_range;
    rclcpp::Parameter p_angle_disable_min, p_angle_disable_max;
    rclcpp::Parameter p_frequency, p_publish_point_cloud;
    rclcpp::Parameter p_use_gps_ts, p_fixed_frame_id, p_child_frame_id;

    rclcpp::Subscription<n301_lidar_ros2::msg::LslidarN301Packet>::SharedPtr packet_sub;
    rclcpp::Publisher<n301_lidar_ros2::msg::LslidarN301Sweep>::SharedPtr sweep_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;

    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group;

    n301_lidar_ros2::msg::LslidarN301Sweep::UniquePtr sweep_data;
    sensor_msgs::msg::PointCloud2 point_cloud_data;
};

}

POINT_CLOUD_REGISTER_POINT_STRUCT(lslidar_n301_decoder::PointXYZIT,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                          uint8_t, intensity,
                                          intensity)(double, timestamp, timestamp))

#endif // LSLIDAR_n301_DECODER_H