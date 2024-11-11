cmake_minimum_required(VERSION 3.8)
project(group7_final)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mage_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include/group7_final
${mage_msgs_INCLUDE_DIRS}
${tf2_INCLUDE_DIRS}
${geometry_msgs_LIBRARIES})

add_executable(camera_processor
  src/camera_processor.cpp
  src/robot_controller.cpp
  src/main.cpp
)
install(TARGETS camera_processor  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(camera_processor
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${mage_msgs_LIBRARIES}
  ${tf2_ros}
  ${tf2_geometry_msgs}
)

ament_target_dependencies(camera_processor 
rclcpp
rcutils
std_msgs
mage_msgs
nav_msgs
geometry_msgs
tf2_ros
tf2
tf2_geometry_msgs)

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

ament_package()
