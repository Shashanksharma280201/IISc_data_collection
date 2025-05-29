cmake_minimum_required(VERSION 3.8)
project(flo_utils)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(flo_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
    flo_utils/__init__.py
    flo_utils/joystick.py
    flo_utils/waypoint_mission_client.py
    flo_utils/ros2_utils.py
    flo_utils/s3_uploader.py
    flo_utils/record_path.py
    flo_utils/path_client.py
    flo_utils/annotate_cmdvel.py
    flo_utils/rtcm_parser.py
    flo_utils/ntrip_client.py
    flo_utils/utils.py
    flo_utils/monitors/perf_monitor.py
    flo_utils/monitors/internet_connectivity_monitor.py
    flo_utils/monitors/lte_monitor.py
    flo_utils/monitors/rosout_monitor.py
    flo_utils/monitors/rtcm_monitor.py
    DESTINATION lib/${PROJECT_NAME})

ament_package()
