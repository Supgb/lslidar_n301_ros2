#include "n301_lidar_ros2/n301_lidar_ros2_driver.hpp"
#include "n301_lidar_ros2/n301_lidar_ros2_decoder.hpp"
#include "rclcpp/rclcpp.hpp"

int main (int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executors;
    
    auto decoder = std::make_shared<lslidar_n301_decoder::LslidarDecoder>("decoder");
    auto driver = std::make_shared<lslidar_n301_driver::LslidarDriver>("driver");

    executors.add_node(decoder);
    executors.add_node(driver);

    while (driver->polling()) {
        executors.spin_once();
    }
    
    //executors.spin();
    rclcpp::shutdown();
    return 0;
}