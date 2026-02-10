cmake_minimum_required(VERSION 3.8)
project(exhibition_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/EmergencyStop.srv"
  "srv/TestSpin.srv"
  "srv/GetStatus.srv"
  DEPENDENCIES builtin_interfaces
)

ament_package()
