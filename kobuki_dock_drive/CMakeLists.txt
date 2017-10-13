##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(kobuki_dock_drive)

##############################################################################
# Find Packages
##############################################################################

find_package(ament_cmake REQUIRED)
find_package(ecl_build REQUIRED)
find_package(ecl_threads REQUIRED)
find_package(ecl_geometry REQUIRED)
find_package(ecl_linear_algebra REQUIRED)

##############################################################################
# Project Configuration
##############################################################################

ecl_enable_cxx11_compiler()
include_directories(include
  ${ecl_build_INCLUDE_DIRS}
  ${ecl_threads_INCLUDE_DIRS}
  ${ecl_geometry_INCLUDE_DIRS}
  ${ecl_linear_algebra_INCLUDE_DIRS}
  )

##############################################################################
# Sources
##############################################################################

add_library(kobuki_dock_drive src/dock_drive.cpp src/dock_drive_states.cpp src/dock_drive_debug.cpp)
target_link_libraries(kobuki_dock_drive
  ${ecl_build_LIBRARIES}
  ${ecl_threads_LIBRARIES}
  ${ecl_geometry_LIBRARIES}
  ${ecl_linear_algebra_LIBRARIES}
)

##############################################################################
# Installs
##############################################################################

install(TARGETS kobuki_dock_drive
        DESTINATION lib
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include
)
