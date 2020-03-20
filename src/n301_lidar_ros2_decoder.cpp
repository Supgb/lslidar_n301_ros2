#include "n301_lidar_ros2/n301_lidar_ros2_decoder.hpp"

namespace lslidar_n301_decoder {

LslidarDecoder::LslidarDecoder(const std::string& name, const int& _point_num, const double& _min_range,
                                const double& _max_range, const double& _angle_disable_min, 
                                const double& _angle_disable_max, const double& _frequency,
                                const std::string& _fixed_frame_id, const std::string& _child_frame_id,
                                const FLAG& _FLAG):
    Node(name, rclcpp::NodeOptions().use_intra_process_comms(true)),
    pTime(), packet_timestamp(), sweep_end_time_gps(),
    sweep_end_time_hardware(), point_num(_point_num), angle_base(),
    min_range(_min_range), max_range(_max_range), angle_disable_min(_angle_disable_min), angle_disable_max(_angle_disable_max), frequency(_frequency), 
    publish_point_cloud((_FLAG&0x01) == 0x01), use_gps_ts((_FLAG&0x02) == 0x02),
    is_first_sweep(true), last_azimuth(0.0), sweep_start_time(0.0),
    packet_start_time(0.0), fixed_frame_id(_fixed_frame_id),  child_frame_id(_child_frame_id),
    p_point_num("point_num", point_num), p_min_range("min_range", min_range),
    p_max_range("max_range", max_range), p_angle_disable_min("angle_disable_min", angle_disable_min),
    p_angle_disable_max("angle_disable_max", angle_disable_max),
    p_frequency("frequency", frequency), p_publish_point_cloud("publish_point_cloud", publish_point_cloud),
    p_use_gps_ts("use_gps_ts", use_gps_ts), p_fixed_frame_id("fixed_frame_id", fixed_frame_id),
    p_child_frame_id("child_frame_id", child_frame_id),
    packet_sub(this->create_subscription<n301_lidar_ros2::msg::LslidarN301Packet>(
        "lslidar_packet", 100,
        std::bind(&LslidarDecoder::packetCallback, this, std::placeholders::_1))), 
    sweep_pub(this->create_publisher<n301_lidar_ros2::msg::LslidarN301Sweep>(
        "lslidar_sweep", 10)), 
    point_cloud_pub(this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "lslidar_point_cloud", 10)), 
    scan_pub(this->create_publisher<sensor_msgs::msg::LaserScan>(
        "scan", 100)),
    callback_group(),
    sweep_data(new n301_lidar_ros2::msg::LslidarN301Sweep()), point_cloud_data() {

        if (!this->initialize()) {
            RCLCPP_ERROR(this->get_logger(), "Could not initialize the decoder...");
        }
        RCLCPP_INFO(this->get_logger(), "Successfully initialize decoder...");
    }


bool LslidarDecoder::loadParameters() {
    std::vector<std::string> parameters = {
        "point_num", "min_range", "max_range", "angle_disable_min",
        "angle_disable_max", "frequency", "publish_point_cloud",
        "use_gps_ts", "fixed_frame_id", "child_frame_id"
    };
    std::for_each(parameters.begin(), parameters.end(), [&](std::string s) {
        this->declare_parameter(s);
    });

    this->set_parameter(p_point_num);
    this->set_parameter(p_min_range);
    this->set_parameter(p_max_range);
    this->set_parameter(p_angle_disable_min);
    this->set_parameter(p_angle_disable_max);
    this->set_parameter(p_frequency);
    this->set_parameter(p_publish_point_cloud);
    this->set_parameter(p_use_gps_ts);
    this->set_parameter(p_fixed_frame_id);
    this->set_parameter(p_child_frame_id);

    angle_base = M_PI*2 / point_num;

    RCLCPP_WARN_STREAM(this->get_logger(), "Using GPS timestamp or not " << use_gps_ts);
    return true;
}

bool LslidarDecoder::initialize() {
    if (!loadParameters()) {
        RCLCPP_WARN(this->get_logger(), "Cannot load all required parameters...");
        return false;
    }

    // Fill in the altitude for each scan.
    for (size_t scan_idx = 0; scan_idx < 16; ++scan_idx) {
        size_t remapped_scan_idx = (scan_idx%2 == 0 ? (scan_idx/2) : (scan_idx/2+8));
        sweep_data->scans[remapped_scan_idx].altitude = scan_altitude[scan_idx];
    }

    // Create the sin and cos table for different azimuth values.
    for (size_t i = 0; i < 6300; ++i) {
        double angle = static_cast<double>(i) / 1000.0;
        cos_azimuth_table[i] = cos(angle);
        sin_azimuth_table[i] = sin(angle);
    }

    return true;
}

bool LslidarDecoder::checkPacketValidity(const RawPacket* packet) {
    for (size_t blk_idx = 0; blk_idx < BLOCKS_PER_PACKET; ++blk_idx) {
        if (packet->blocks[blk_idx].header != UPPER_BANK) {
            RCLCPP_WARN(this->get_logger(), "Skip invalid LS-16 packet: block %lu header is %x",
                     blk_idx, packet->blocks[blk_idx].header);
            return false;
        }
    }
    return true;
}

/* void LslidarDecoder::publishPointCloud() {
    auto point_cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    auto pcl_pc = std::make_unique<pcl::PointCloud<pcl::PointXYZI> >();
    
    for (size_t i = 0; i < 16; ++i) {
        const n301_lidar_ros2::msg::LslidarN301Scan& scan = sweep_data->scans[i];
        if (scan.points.size() == 0) continue;
        for (size_t j = 1; j < scan.points.size()-1; ++j) {
            pcl::PointXYZI point;
            //point.timestamp = timestamp - (scan.points.size()-1 -j)*0.05;
            point.x = scan.points[j].x;
            point.y = scan.points[j].y;
            point.z = scan.points[j].z;
            point.intensity = scan.points[j].intensity;
            pcl_pc->points.push_back(point);
            ++pcl_pc->width;
        }
    }
    pcl_pc->header.stamp = pcl_conversions::toPCL(sweep_data->header).stamp;
    pcl::toROSMsg(*pcl_pc, *point_cloud);
    double timestamp = sweep_data->header.stamp.sec;
    // point_cloud->header.stamp = pcl_conversions::toPCL(sweep_data->header).stamp;
    point_cloud->header.frame_id = child_frame_id;
    point_cloud->height = 1;

    point_cloud_pub->publish(std::move(point_cloud));
} */

void LslidarDecoder::publishScan() {
    auto scan = sensor_msgs::msg::LaserScan::UniquePtr(
        new sensor_msgs::msg::LaserScan());
    
    if (sweep_data->scans[0].points.size() <=1 ) return;

    //time_t tick = static_cast<time_t>(sweep_end_time_gps);
    //struct tm tm = *localtime(&tick);
    
    scan->header.frame_id = child_frame_id;
    scan->header.stamp = sweep_data->header.stamp;
    scan->angle_min = 0.0;
    scan->angle_max = 2.0 * M_PI;
    scan->angle_increment = (scan->angle_max - scan->angle_min)/point_num;
    scan->time_increment = 0.05;    // s
    scan->scan_time = 1/10.0;   // s
    scan->range_min = min_range;
    scan->range_max = max_range;
    scan->ranges.reserve(point_num);
    scan->ranges.assign(point_num, std::numeric_limits<float>::infinity());
    scan->intensities.reserve(point_num);
    scan->intensities.assign(point_num, std::numeric_limits<float>::infinity());
    
    for (uint16_t i = 0; i < sweep_data->scans[0].points.size(); i++) {
        int point_idx = static_cast<int>(sweep_data->scans[0].points[i].azimuth / angle_base);
        
        if (point_idx >= point_num)
            point_idx = 0;
        if (point_idx < 0)
            point_idx = point_num - 1;
        scan->ranges[point_num-1-point_idx] = sweep_data->scans[0].points[i].distance;
        scan->intensities[point_num-1-point_idx] = sweep_data->scans[0].points[i].intensity;
    }

    for (int i = point_num -1; i >= 0; i--) {
        if ((i >= angle_disable_min*point_num/360) && (i < angle_disable_max*point_num/360))
            scan->ranges[i] = std::numeric_limits<float>::infinity();
    }
    
    scan_pub->publish(std::move(scan));
}

point_struct LslidarDecoder::getMeans(std::vector<point_struct> clusters) {
    point_struct tmp;
    int num = clusters.size();
    if (num == 0) {
        tmp.distance = std::numeric_limits<float>::infinity();
        tmp.intensity = std::numeric_limits<float>::infinity();
    } else {
        double mean_distance = 0, mean_intensity = 0;
        for (int i = 0; i < num; i++) {
            mean_distance += clusters[i].distance;
            mean_intensity += clusters[i].intensity;
        }
        tmp.distance = mean_distance / num;
        tmp.intensity = mean_intensity /num;
    }
    return tmp;
}

uint64_t LslidarDecoder::get_gps_stamp(struct tm t) {
    uint64_t ptime = static_cast<uint64_t>(timegm(&t));
    return ptime;
}

void LslidarDecoder::decodePacket(const RawPacket* packet) {
    // Compute the azimuth angle for each firing
    for (size_t fir_idx = 0; fir_idx < FIRINGS_PER_PACKET; fir_idx+=2) {
        size_t blk_idx = fir_idx / 2;
        firings[fir_idx].firing_azimuth = rawAzimuthToDouble(
            packet->blocks[blk_idx].rotation);
    }
    // Interpolate the azimuth values
    for (size_t fir_idx = 1; fir_idx < FIRINGS_PER_PACKET; fir_idx += 2) {
        size_t lfir_idx = fir_idx - 1;
        size_t rfir_idx = fir_idx + 1;

        double azimuth_diff;
        if (fir_idx == FIRINGS_PER_PACKET - 1) {
            lfir_idx = fir_idx - 3;
            rfir_idx = fir_idx - 1;
        }
        
        azimuth_diff = firings[rfir_idx].firing_azimuth - firings[lfir_idx].firing_azimuth;
        azimuth_diff = azimuth_diff < 0 ? azimuth_diff + 2*M_PI : azimuth_diff;

        firings[fir_idx].firing_azimuth = firings[fir_idx-1].firing_azimuth + azimuth_diff/2.0;
        firings[fir_idx].firing_azimuth = firings[fir_idx].firing_azimuth > 2*M_PI ?
            firings[fir_idx].firing_azimuth-2*M_PI : firings[fir_idx].firing_azimuth;
    }
        // Fill in the distance and intensity for each firing.
    for (size_t blk_idx = 0; blk_idx < BLOCKS_PER_PACKET; ++blk_idx) {
        const RawBlock& raw_block = packet->blocks[blk_idx];

        for (size_t blk_fir_idx = 0; blk_fir_idx < FIRINGS_PER_BLOCK; ++blk_fir_idx){
            size_t fir_idx = blk_idx*FIRINGS_PER_BLOCK + blk_fir_idx;

            double azimuth_diff = 0.0;  
            if (fir_idx < FIRINGS_PER_PACKET - 1)              
                azimuth_diff = firings[fir_idx+1].firing_azimuth - firings[fir_idx].firing_azimuth;
            else
                azimuth_diff = firings[fir_idx].firing_azimuth - firings[fir_idx-1].firing_azimuth;

            for (size_t scan_fir_idx = 0; scan_fir_idx < SCANS_PER_FIRING; ++scan_fir_idx){
                size_t byte_idx = RAW_SCAN_SIZE * (SCANS_PER_FIRING*blk_fir_idx + scan_fir_idx);
                // Azimuth
                firings[fir_idx].azimuth[scan_fir_idx] = firings[fir_idx].firing_azimuth +
                        (scan_fir_idx*DSR_TOFFSET/FIRING_TOFFSET) * azimuth_diff;
                // Distance
                TwoBytes raw_distance;
                raw_distance.bytes[0] = raw_block.data[byte_idx];
                raw_distance.bytes[1] = raw_block.data[byte_idx+1];
                firings[fir_idx].distance[scan_fir_idx] = static_cast<double>(
                            raw_distance.distance) * DISTANCE_RESOLUTION;
                // Intensity
                firings[fir_idx].intensity[scan_fir_idx] = static_cast<double>(
                            raw_block.data[byte_idx+2]);
            }
            //extract gps_time
            pTime.tm_year = raw_block.data[90]+2000-1900;
            pTime.tm_mon = raw_block.data[91]-1;
            pTime.tm_mday = raw_block.data[92];
            pTime.tm_hour = raw_block.data[93];
            pTime.tm_min = raw_block.data[94];
            pTime.tm_sec = raw_block.data[95];        
        }     
    }
    // resolve the timestamp in the end of packet
    packet_timestamp = (packet->time_stamp_yt[0]  + 
                        packet->time_stamp_yt[1] * pow(2, 8) +
                        packet->time_stamp_yt[2] * pow(2, 16) + 
                        packet->time_stamp_yt[3] * pow(2, 24)) * 1e3;
}

void LslidarDecoder::packetCallback(n301_lidar_ros2::msg::LslidarN301Packet::UniquePtr msg) {
    const RawPacket *raw_packet = reinterpret_cast<const RawPacket*>(&(msg->data[0]));
    if (!checkPacketValidity(raw_packet)) return;
    decodePacket(raw_packet);   // Decode.

    size_t new_sweep_start = 0;
    do {
        if (fabs(firings[new_sweep_start].firing_azimuth - last_azimuth) > M_PI) break;
        else {
            last_azimuth = firings[new_sweep_start].firing_azimuth;
            ++new_sweep_start;
        }
    } while (new_sweep_start < FIRINGS_PER_PACKET);

    // The first sweep may not be complete. So, the firings with
    // the first sweep will be discarded. We will wait for the
    // second sweep in order to find the 0 azimuth angle.
    size_t start_fir_idx = 0;
    size_t end_fir_idx = new_sweep_start;
    if (is_first_sweep &&
            new_sweep_start == FIRINGS_PER_PACKET) {
        // The first sweep has not ended yet.
        return;
    } else {
        if (is_first_sweep) {
            is_first_sweep = false;
            start_fir_idx = new_sweep_start;
            end_fir_idx = FIRINGS_PER_PACKET;
            // sweep_start_time = msg->stamp.toSec() +
                    // FIRING_TOFFSET * (end_fir_idx-start_fir_idx) * 1e-6;
            sweep_start_time = get_gps_stamp(pTime);
        }
    }

    for (size_t fir_idx = start_fir_idx; fir_idx < end_fir_idx; ++fir_idx) {
        for (size_t scan_idx = 0; scan_idx < SCANS_PER_FIRING; ++scan_idx) {
            // Check if the point is valid.
            if (!isPointInRange(firings[fir_idx].distance[scan_idx])) continue;

            // Convert the point to xyz coordinate
            size_t table_idx = floor(firings[fir_idx].azimuth[scan_idx]*1000.0+0.5);
            //cout << table_idx << endl;
            double cos_azimuth = cos_azimuth_table[table_idx];
            double sin_azimuth = sin_azimuth_table[table_idx];

            //double x = firings[fir_idx].distance[scan_idx] *
            //  cos_scan_altitude[scan_idx] * sin(firings[fir_idx].azimuth[scan_idx]);
            //double y = firings[fir_idx].distance[scan_idx] *
            //  cos_scan_altitude[scan_idx] * cos(firings[fir_idx].azimuth[scan_idx]);
            //double z = firings[fir_idx].distance[scan_idx] *
            //  sin_scan_altitude[scan_idx];

            double x = firings[fir_idx].distance[scan_idx] *
                    cos_scan_altitude[scan_idx] * sin_azimuth;
            double y = firings[fir_idx].distance[scan_idx] *
                    cos_scan_altitude[scan_idx] * cos_azimuth;
            double z = firings[fir_idx].distance[scan_idx] *
                    sin_scan_altitude[scan_idx];

            double x_coord = y;
            double y_coord = -x;
            double z_coord = z;

            // Compute the time of the point
            // double time = packet_start_time +
                    // FIRING_TOFFSET*fir_idx + DSR_TOFFSET*scan_idx;

            // get timestamp from gps and hardware instead
            double time = sweep_end_time_gps*1.0 + sweep_end_time_hardware*1e-9;

            // Remap the index of the scan
            int remapped_scan_idx = scan_idx%2 == 0 ? scan_idx/2 : scan_idx/2+8;
            sweep_data->scans[remapped_scan_idx].points.push_back(
                        n301_lidar_ros2::msg::LslidarN301Point());

            n301_lidar_ros2::msg::LslidarN301Point& new_point =		// new_point 为push_back最后一个的引用
                    sweep_data->scans[remapped_scan_idx].points[
                    sweep_data->scans[remapped_scan_idx].points.size()-1];

            // Pack the data into point msg
            new_point.time = time;
            new_point.x = x_coord;
            new_point.y = y_coord;
            new_point.z = z_coord;
            new_point.azimuth = firings[fir_idx].azimuth[scan_idx];
            new_point.distance = firings[fir_idx].distance[scan_idx];
            new_point.intensity = firings[fir_idx].intensity[scan_idx];
        }
    }

    packet_start_time += FIRING_TOFFSET * (end_fir_idx-start_fir_idx);

    // A new sweep begins
    if (end_fir_idx != FIRINGS_PER_PACKET) {
        //	ROS_WARN("A new sweep begins");
        // Publish the last revolution
        sweep_end_time_gps = get_gps_stamp(pTime);
        sweep_end_time_hardware = packet_timestamp;

        sweep_data->header.frame_id = "sweep";
        if (use_gps_ts){
            //sweep_data->header.stamp = ros::Time(sweep_end_time_gps, sweep_end_time_hardware);
        }
        else{
            sweep_data->header.stamp = this->now();
        }

        //if (publish_point_cloud) publishPointCloud();
        
        publishScan();

        sweep_pub->publish(std::move(sweep_data));

        sweep_data = n301_lidar_ros2::msg::LslidarN301Sweep::UniquePtr(
            new n301_lidar_ros2::msg::LslidarN301Sweep());

        // Prepare the next revolution
        // sweep_start_time = msg->stamp.toSec() +
        //         FIRING_TOFFSET * (end_fir_idx-start_fir_idx) * 1e-6;
        sweep_start_time = sweep_end_time_gps * 1.0 + sweep_end_time_hardware*1e-9;

        packet_start_time = 0.0;
        last_azimuth = firings[FIRINGS_PER_PACKET-1].firing_azimuth;

        start_fir_idx = end_fir_idx;
        end_fir_idx = FIRINGS_PER_PACKET;

        for (size_t fir_idx = start_fir_idx; fir_idx < end_fir_idx; ++fir_idx) {
            for (size_t scan_idx = 0; scan_idx < SCANS_PER_FIRING; ++scan_idx) {
                // Check if the point is valid.
                if (!isPointInRange(firings[fir_idx].distance[scan_idx])) continue;

                // Convert the point to xyz coordinate
                size_t table_idx = floor(firings[fir_idx].azimuth[scan_idx]*1000.0+0.5);
                //cout << table_idx << endl;
                double cos_azimuth = cos_azimuth_table[table_idx];
                double sin_azimuth = sin_azimuth_table[table_idx];

                //double x = firings[fir_idx].distance[scan_idx] *
                //  cos_scan_altitude[scan_idx] * sin(firings[fir_idx].azimuth[scan_idx]);
                //double y = firings[fir_idx].distance[scan_idx] *
                //  cos_scan_altitude[scan_idx] * cos(firings[fir_idx].azimuth[scan_idx]);
                //double z = firings[fir_idx].distance[scan_idx] *
                //  sin_scan_altitude[scan_idx];

                double x = firings[fir_idx].distance[scan_idx] *
                        cos_scan_altitude[scan_idx] * sin_azimuth;
                double y = firings[fir_idx].distance[scan_idx] *
                        cos_scan_altitude[scan_idx] * cos_azimuth;
                double z = firings[fir_idx].distance[scan_idx] *
                        sin_scan_altitude[scan_idx];

                double x_coord = y;
                double y_coord = -x;
                double z_coord = z;

                // Compute the time of the point
                // double time = packet_start_time +
                        // FIRING_TOFFSET*(fir_idx-start_fir_idx) + DSR_TOFFSET*scan_idx;

                // get timestamp from gps and hardware instead
                double time = sweep_end_time_gps*1.0 + sweep_end_time_hardware*1e-9;

                // Remap the index of the scan
                int remapped_scan_idx = scan_idx%2 == 0 ? scan_idx/2 : scan_idx/2+8;
                sweep_data->scans[remapped_scan_idx].points.push_back(
                            n301_lidar_ros2::msg::LslidarN301Point());
                n301_lidar_ros2::msg::LslidarN301Point& new_point =
                        sweep_data->scans[remapped_scan_idx].points[
                        sweep_data->scans[remapped_scan_idx].points.size()-1];

                // Pack the data into point msg
                new_point.time = time;
                new_point.x = x_coord;
                new_point.y = y_coord;
                new_point.z = z_coord;
                new_point.azimuth = firings[fir_idx].azimuth[scan_idx];
                new_point.distance = firings[fir_idx].distance[scan_idx];
                new_point.intensity = firings[fir_idx].intensity[scan_idx];
            }
        }

        packet_start_time += FIRING_TOFFSET * (end_fir_idx-start_fir_idx);
    }
}

}   // namespace lslidar_n301_decoder