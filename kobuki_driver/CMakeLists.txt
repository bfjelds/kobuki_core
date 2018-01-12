##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(kobuki_driver)

##############################################################################
# Catkin
##############################################################################

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(ecl_build REQUIRED)
find_package(ecl_mobile_robot REQUIRED)
find_package(ecl_converters REQUIRED)
find_package(ecl_devices REQUIRED)
find_package(ecl_geometry REQUIRED)
find_package(ecl_sigslots REQUIRED)
find_package(ecl_time REQUIRED)
find_package(ecl_command_line REQUIRED)
find_package(kobuki_msgs REQUIRED)

##############################################################################
# Project Configuration
##############################################################################

ecl_enable_cxx11_compiler()
include_directories(include 
  ${rcl_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rcutils_INCLUDE_DIRS}
  ${ecl_build_INCLUDE_DIRS}
  ${ecl_mobile_robot_INCLUDE_DIRS}
  ${ecl_converters_INCLUDE_DIRS}
  ${ecl_devices_INCLUDE_DIRS}
  ${ecl_geometry_INCLUDE_DIRS}
  ${ecl_sigslots_INCLUDE_DIRS}
  ${ecl_time_INCLUDE_DIRS}
  ${ecl_command_line_INCLUDE_DIRS}
  ${kobuki_msgs_INCLUDE_DIRS}
  )

##############################################################################
# Sources
##############################################################################

add_subdirectory(src)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/kobuki_driver
)


ament_export_include_directories(include/kobuki_driver)
ament_export_dependencies(rcl)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rcutils)
ament_export_dependencies(ecl_time)
ament_export_dependencies(ecl_devices)
ament_export_dependencies(ecl_geometry)
ament_export_dependencies(ecl_mobile_robot)
ament_export_dependencies(kobuki_msgs)
ament_export_libraries(kobuki)
ament_package()
