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

add_executable(serial_port src/serial_port.cpp)
target_include_directories(serial_port PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${SERIAL_INCLUDE_DIRS}
)
target_link_libraries(serial_port
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${SERIAL_LDFLAGS}
)

install(TARGETS
    serial_port
    DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
    flo_driver/flo_base_diff.py
    flo_driver/oakd.py
    DESTINATION lib/${PROJECT_NAME})
    
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME})

ament_package()
