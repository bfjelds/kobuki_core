##############################################################################
# SOURCES
##############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

##############################################################################
# CONFIGURATIONS
##############################################################################

set(VERSION_FILE ${PROJECT_SOURCE_DIR}/build/version_info.cpp)
configure_file(version_info.cpp.in ${VERSION_FILE} @ONLY)

##############################################################################
# LIBRARIES
##############################################################################

add_library(kobuki ${SOURCES} ${VERSION_FILE})
target_link_libraries(kobuki 
  ${rcl_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${rcutils_LIBRARIES}
  ${ecl_build_LIBRARIES}
  ${ecl_mobile_robot_LIBRARIES}
  ${ecl_converters_LIBRARIES}
  ${ecl_devices_LIBRARIES}
  ${ecl_geometry_LIBRARIES}
  ${ecl_sigslots_LIBRARIES}
  ${ecl_time_LIBRARIES}
  ${ecl_command_line_LIBRARIES}
  ${kobuki_msgs_LIBRARIES}
  )

install(TARGETS kobuki
        DESTINATION lib
)

