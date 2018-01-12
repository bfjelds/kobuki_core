###############################################################################
# Tools
###############################################################################

add_executable(version_info version_info.cpp)
target_link_libraries(version_info kobuki 
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
install(TARGETS version_info
        DESTINATION bin
)

if (NOT ${MSVC})
add_executable(simple_keyop simple_keyop.cpp)
target_link_libraries(simple_keyop kobuki 
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
install(TARGETS simple_keyop
        DESTINATION bin
)
endif()
