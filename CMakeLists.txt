cmake_minimum_required(VERSION 3.5)
project(n301_lidar_ros2)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)

set(msg_files
  "msg/LslidarN301Packet.msg"
  "msg/LslidarN301Point.msg"
  "msg/LslidarN301Scan.msg"
  "msg/LslidarN301Sweep.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

add_library(n301_lidar_ros2_driver
  src/n301_lidar_ros2_driver.cpp
)
ament_target_dependencies(n301_lidar_ros2_driver
  rclcpp std_msgs sensor_msgs pluginlib
  pcl_conversions tf2_ros PCL Boost
  builtin_interfaces
)
get_default_rmw_implementation(rmw_implementation)
find_package("${rmw_implementation}" REQUIRED)
get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")

foreach(typesupport_impl ${typesupport_impls})
  rosidl_target_interfaces(n301_lidar_ros2_driver
    ${PROJECT_NAME} ${typesupport_impl}
  )
endforeach()

add_library(n301_lidar_ros2_decoder
  src/n301_lidar_ros2_decoder.cpp
)
ament_target_dependencies(n301_lidar_ros2_decoder
  rclcpp std_msgs sensor_msgs pluginlib
  pcl_conversions tf2_ros PCL Boost
  builtin_interfaces
)
foreach(typesupport_impl ${typesupport_impls})
  rosidl_target_interfaces(n301_lidar_ros2_decoder
    ${PROJECT_NAME} ${typesupport_impl}
  )
endforeach()

add_executable(n301_node
  src/n301_node.cpp
)
target_link_libraries(n301_node
  n301_lidar_ros2_driver
  n301_lidar_ros2_decoder
)
ament_target_dependencies(n301_node
  rclcpp std_msgs sensor_msgs pluginlib
  pcl_conversions tf2_ros PCL Boost
  builtin_interfaces
)
foreach(typesupport_impl ${typesupport_impls})
  rosidl_target_interfaces(n301_node
    ${PROJECT_NAME} ${typesupport_impl}
  )
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  n301_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)

ament_package()
