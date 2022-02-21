cmake_minimum_required(VERSION 3.0.2)
add_compile_options(-std=c++17 -Wall -Wextra -Wpedantic -Werror)
project(path_tracking_pid)

## Find catkin and any catkin packages
find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    actionlib_msgs
    dynamic_reconfigure
    geometry_msgs
    mbf_costmap_core
    mbf_msgs
    message_generation
    nav_core
    nav_msgs
    pluginlib
    roscpp
    roslint
    rostest
    std_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)

add_message_files(
  FILES
    PidDebug.msg
    PidFeedback.msg
)

generate_dynamic_reconfigure_options(
  cfg/Pid.cfg
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    nav_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    dynamic_reconfigure
    geometry_msgs
    mbf_costmap_core
    mbf_msgs
    message_runtime
    nav_msgs
    pluginlib
    roscpp
    std_msgs
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}_local_planner.cpp
  src/controller.cpp
  src/visualization.cpp
  src/details/second_order_lowpass.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
include_directories(include)

# Configure roslint for nodes

# Roslint cpp
set(ROSLINT_CPP_OPTS "--filter=-legal/copyright")
roslint_cpp()

install(
  TARGETS
    ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install files
install(
  FILES
    README.md
    ${PROJECT_NAME}_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install directories
install(
  DIRECTORY
    doc
    launch
    param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  add_rostest(test/test_path_tracking_pid.test ARGS rviz:=false reconfigure:=false)
endif()
