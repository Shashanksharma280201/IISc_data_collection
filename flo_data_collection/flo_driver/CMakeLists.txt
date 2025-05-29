cmake_minimum_required(VERSION 3.8)
project(flo_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(flo_msgs REQUIRED)
find_package(flo_utils REQUIRED)

ament_python_install_package(${PROJECT_NAME})

include_directories(
  include
)


install(PROGRAMS
    flo_driver/flo_base_diff.py
    flo_driver/flysky_controller.py
    flo_driver/oakd.py
    flo_driver/nmea_topic_driver.py
    flo_driver/serial_port_read.py
    flo_driver/ps4_controller_utils.py
    flo_driver/ps4_controller.py
    flo_driver/ntrip.py
    DESTINATION lib/${PROJECT_NAME})
    
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME})

ament_package()
