cmake_minimum_required(VERSION 3.8)
project(map_scan_integrator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

add_executable(map_scan_integrator
    src/scan.cpp
    src/map.cpp
    src/map_scan_integrator.cpp
    src/tf.cpp
    src/communication.cpp
    src/main.cpp
)

ament_target_dependencies(map_scan_integrator
    ament_cmake
    rclcpp
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
	nav_msgs
    geometry_msgs
)

target_include_directories(map_scan_integrator PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

install(TARGETS map_scan_integrator
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
   	launch
	rviz
    param
	DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
	include/${PROJECT_NAME}
	DESTINATION include
)
ament_package()
