cmake_minimum_required(VERSION 3.8)
project(path_tracking_pid)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

nav2_package()
set(CMAKE_CXX_STANDARD 17)

include_directories(
  include
)

set(dependencies
  builtin_interfaces
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  rclcpp
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PidDebug.msg"
  "msg/PidFeedback.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

set(library_name path_tracking_pid_lib)

add_library(${library_name} SHARED
        src/path_tracking_pid_nav2_controller.cpp
        src/controller.cpp)

# prevent pluginlib from using boost
target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(${library_name}
  ${dependencies}
)

# rosidl_target_interfaces(${library_name}  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(${library_name} "${cpp_typesupport_target}")

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core path_tracking_pid.xml)

ament_package()
