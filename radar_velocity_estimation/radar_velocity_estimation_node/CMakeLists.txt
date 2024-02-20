cmake_minimum_required(VERSION 3.8)
project(radar_velocity_estimation_node)

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

find_package(radar_velocity_estimation)

include_directories(${EIGEN3_INCLUDE_DIR})

set(PROJECT_DEPENDENCIES rclcpp sensor_msgs geometry_msgs tf2_ros std_srvs radar_velocity_estimation)

set(PROJECT_SOURCE
        src/RadarVelocityNode.cpp
        include/radar_velocity_estimation_node/RadarVelocityNode.h
        include/radar_velocity_estimation_node/topics.h
        src/ros_radar_utils.cpp
        include/radar_velocity_estimation_node/ros_radar_utils.h)


add_executable(radar_velocity_estimation_node src/radar_velocity_estimation_node.cpp ${PROJECT_SOURCE})

target_include_directories(radar_velocity_estimation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(radar_velocity_estimation_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(radar_velocity_estimation_node ${PROJECT_DEPENDENCIES})
target_link_libraries(radar_velocity_estimation_node gtsam)

add_library(radar_velocity_estimation_node_lib ${PROJECT_SOURCE})
target_include_directories(radar_velocity_estimation_node_lib PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(radar_velocity_estimation_node_lib PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(radar_velocity_estimation_node_lib ${PROJECT_DEPENDENCIES})
ament_export_dependencies(${PROJECT_DEPENDENCIES})

install(TARGETS radar_velocity_estimation_node
  DESTINATION lib/${PROJECT_NAME})

install(
        DIRECTORY include/
        DESTINATION include
)

install(
        TARGETS radar_velocity_estimation_node_lib
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
        radar_velocity_estimation_node_lib
)
ament_export_targets(
        export_${PROJECT_NAME}
)

ament_package()
