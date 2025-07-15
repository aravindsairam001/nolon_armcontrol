cmake_minimum_required(VERSION 3.8)
project(nolon_robot_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_visual_tools REQUIRED)

# Add executables
# add_executable(toilet_scene_node src/toilet_scene_node.cpp)
# add_executable(robotAMR src/robotAMR.cpp)
# add_executable(mtc_spray 
#   src/mtc_spray.cpp
#   src/toilet_scene.cpp
# )

add_executable(mtc_spray_clean 
  src/mtc_spray_clean.cpp
  src/toilet_scene.cpp
)

# target_include_directories(mtc_spray PRIVATE
#   ${CMAKE_CURRENT_SOURCE_DIR}/include
# )

target_include_directories(mtc_spray_clean PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link dependencies for robotAMR
# ament_target_dependencies(robotAMR
#   rclcpp
#   moveit_core
#   moveit_ros_planning_interface
#   geometric_shapes
# )

# ament_target_dependencies(toilet_scene_node
#   rclcpp
#   moveit_ros_planning_interface
#   geometry_msgs
# )

# ament_target_dependencies(mtc_spray
#   rclcpp
#   moveit_core
#   moveit_task_constructor_core
#   moveit_ros_planning_interface
#   geometry_msgs
#   moveit_visual_tools
#   tf2_geometry_msgs
# )


ament_target_dependencies(mtc_spray_clean
  rclcpp
  moveit_core
  moveit_task_constructor_core
  moveit_ros_planning_interface
  geometry_msgs
  moveit_visual_tools
  tf2_geometry_msgs
)
# Install executables
install(TARGETS   mtc_spray_clean
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install URDF files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install mesh files
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# Install config files (for future use)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()