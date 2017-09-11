cmake_minimum_required(VERSION 2.8.3)
project(uchile_grasp_generator)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  eigen_conversions
  eigen_stl_containers
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_core
  moveit_visual_tools
  tf
  tf_conversions
  trajectory_msgs
  message_generation
  geometry_msgs
  actionlib
  actionlib_msgs
  moveit_msgs
  cmake_modules
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED thread system)

# Catkin
catkin_package(
  LIBRARIES
    uchile_grasp
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    moveit_msgs
    trajectory_msgs
    std_msgs
    message_runtime
    moveit_visual_tools
  INCLUDE_DIRS include
)

# Build
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(uchile_grasp
    src/grasp_generator.cpp
    src/grasp_options.cpp
    src/grasp_filter.cpp
    src/gravitational_torque_estimation.cpp
)
target_link_libraries(uchile_grasp
    ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

# Test grasp executable
add_executable(test_grasp_generator src/test_grasp_generator.cpp)
target_link_libraries(test_grasp_generator ${catkin_LIBRARIES} ${Boost_LIBRARIES} uchile_grasp)

add_executable(test_grasp_filter src/test_grasp_filter.cpp)
target_link_libraries(test_grasp_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES} uchile_grasp)


