cmake_minimum_required(VERSION 3.8)
project(pingdsp_msg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Generate custom messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Ping3DSS.msg"
  "msg/SystemInfo.msg"
  "srv/AppControl.srv"
  "srv/SidescanSettings.srv"
  "srv/Sidescan3DSettings.srv"
  "srv/BathymetrySettings.srv"
  "srv/TransmitSettings.srv"
  "srv/AcquisitionSettings.srv"
  "srv/CommitSettings.srv"
  "srv/SonarControl.srv"
  "srv/FileControl.srv"
  "srv/RecordControl.srv"
  "srv/BaudSettings.srv"
  "srv/SoundVelocity.srv"
  DEPENDENCIES sensor_msgs std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
