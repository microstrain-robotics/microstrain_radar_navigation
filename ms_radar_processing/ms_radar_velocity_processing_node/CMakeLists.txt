cmake_minimum_required(VERSION 3.8)
project(ms_radar_velocity_processing_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp)
find_package(sensor_msgs)
find_package(geometry_msgs)
find_package(tf2_ros)
find_package(std_srvs)

find_package(ms_radar_velocity_processing)

include_directories(${EIGEN3_INCLUDE_DIR})

set(PROJECT_DEPENDENCIES rclcpp sensor_msgs geometry_msgs tf2_ros std_srvs ms_radar_velocity_processing)

set(PROJECT_SOURCE
        src/RadarVelocityNode.cpp
        include/ms_radar_velocity_processing_node/RadarVelocityNode.h
        include/ms_radar_velocity_processing_node/topics.h
        src/ros_radar_utils.cpp
        include/ms_radar_velocity_processing_node/ros_radar_utils.h)


add_executable(ms_radar_velocity_processing_node src/ms_radar_velocity_processing_node.cpp ${PROJECT_SOURCE})

target_include_directories(ms_radar_velocity_processing_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ms_radar_velocity_processing_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(ms_radar_velocity_processing_node ${PROJECT_DEPENDENCIES})
target_link_libraries(ms_radar_velocity_processing_node gtsam)

add_library(ms_radar_velocity_processing_node_lib ${PROJECT_SOURCE})
target_include_directories(ms_radar_velocity_processing_node_lib PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(ms_radar_velocity_processing_node_lib PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(ms_radar_velocity_processing_node_lib ${PROJECT_DEPENDENCIES})
ament_export_dependencies(${PROJECT_DEPENDENCIES})

install(TARGETS ms_radar_velocity_processing_node
  DESTINATION lib/${PROJECT_NAME})

install(
        DIRECTORY include/
        DESTINATION include
)

install(
        TARGETS ms_radar_velocity_processing_node_lib
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
        include
)
ament_export_libraries(
        ms_radar_velocity_processing_node_lib
)
ament_export_targets(
        export_${PROJECT_NAME}
)

ament_package()
