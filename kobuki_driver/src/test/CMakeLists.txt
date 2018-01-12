###############################################################################
# Tests
###############################################################################

add_executable(kobuki_velocity_commands velocity_commands.cpp)
target_link_libraries(kobuki_velocity_commands kobuki 
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

add_executable(demo_kobuki_initialisation initialisation.cpp)
target_link_libraries(demo_kobuki_initialisation kobuki 
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

add_executable(demo_kobuki_sigslots sigslots.cpp)
target_link_libraries(demo_kobuki_sigslots kobuki 
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

add_executable(demo_kobuki_simple_loop simple_loop.cpp)
target_link_libraries(demo_kobuki_simple_loop kobuki 
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

install(TARGETS kobuki_velocity_commands demo_kobuki_initialisation demo_kobuki_sigslots demo_kobuki_simple_loop
        DESTINATION bin
)