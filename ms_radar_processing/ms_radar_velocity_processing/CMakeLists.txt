cmake_minimum_required(VERSION 3.8)
project(ms_radar_velocity_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(Eigen3 3.4.0 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(rclcpp)

include_directories(${EIGEN3_INCLUDE_DIR} ${GTSAM_INCLUDE_DIR})

add_library(ms_radar_velocity_processing
        include/ms_radar_velocity_processing/data.h
        src/calculate_radar_velocity.cpp
        include/ms_radar_velocity_processing/calculate_radar_velocity.h
        src/RadialVelocityFactor.cpp
        src/RadialVelocityFactor.h)
target_compile_features(ms_radar_velocity_processing PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(ms_radar_velocity_processing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(ms_radar_velocity_processing Eigen3 GTSAM rclcpp) 
ament_export_dependencies(Eigen3 GTSAM rclcpp)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(ms_radar_velocity_processing PRIVATE "MS_RADAR_VELOCITY_PROCESSING_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ms_radar_velocity_processing
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
  ms_radar_velocity_processing
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
