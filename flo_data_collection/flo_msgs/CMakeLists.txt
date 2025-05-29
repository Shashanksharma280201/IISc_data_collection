cmake_minimum_required(VERSION 3.8)
project(flo_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Axis.msg"
  "msg/FlySky.msg"
  "msg/Gamepad.msg"
  "msg/JointCmd.msg"
  "msg/JointState.msg"
  "msg/LteInfo.msg"
  "msg/NmeaGGA.msg"
  "msg/NmeaGLL.msg"
  "msg/NmeaGSA.msg"
  "msg/NmeaGSV.msg"
  "msg/NmeaRMC.msg"
  "msg/NmeaVTG.msg"
  "msg/OakdUsage.msg"
  "msg/Perf.msg"
  "msg/Ps4ControllerStatus.msg"
  "msg/ThermalZone.msg"
  "msg/Waypoint.msg"
  "msg/Point2D.msg"
  "msg/Path2D.msg"
  "msg/Pose2D.msg"
  "msg/PingResult.msg"
  "msg/ClockFrequency.msg"
  "msg/Governor.msg"
  "msg/OccupancyGrid.msg"
  "msg/RobotCalibrationStatus.msg"
  "msg/S3UploadStatus.msg"
  "msg/Voltage.msg"
  "srv/StartLogging.srv"
  "srv/StopLogging.srv"
  "srv/Enable.srv"
  "srv/BookTeleopSession.srv"
  "srv/GetAnnotationStatus.srv"
  "srv/GetRobotPose.srv"
  "srv/SetPs4ControllerLed.srv"
  "srv/SetPs4ControllerLedFlash.srv"
  "srv/VibratePs4Controller.srv"
  "srv/StartRobotCalibration.srv"
  "srv/StartRecordingPath.srv"
  "srv/StopRecordingPath.srv"
  "srv/GetSimId.srv"
  "srv/EnableUploading.srv"
  "srv/UpdateUploadParams.srv"
  "action/InplaceRotate.action"
  "action/Waypoint.action"
  "action/WaypointMission.action"
  "action/Path.action"
  "action/PathMission.action"
  DEPENDENCIES std_msgs
)

ament_package()
