cmake_minimum_required(VERSION 2.8.3)
project(bender_arm_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cmake_modules
  dynamic_reconfigure
  interactive_markers
  message_generation
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_simple_grasps
  octomap_msgs
  pluginlib
  roscpp
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  uchile_msgs
  bender_arm_planning
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED system filesystem date_time thread)

find_package(Eigen3 REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Position.srv
  PositionNamed.srv
  PositionServoing.srv
  HeadPosition.srv
  EndEffectorPose.srv
  GripperOrientation.srv
  SetPlannerID.srv
  Grasp.srv
  AttachObject.srv
  JointTarget.srv
)

# Generate actions in the 'action' folder
add_action_files(
  FILES
  GraspGenerator.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  uchile_msgs
  geometry_msgs
  moveit_msgs
  std_msgs
  trajectory_msgs
)

#generate_dynamic_reconfigure_options(
#)
#add_dependencies(example_node ${PROJECT_NAME}_gencfg)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bender_arm_control
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    message_runtime
    moveit_visual_tools
    actionlib_msgs
    bender_arm_planning
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(bender_arm_control
#   src/${PROJECT_NAME}/bender_arm_control.cpp
# )

## Declare a cpp executable
# add_executable(bender_arm_control_node src/bender_arm_control_node.cpp)
add_executable(octomap_pub src/octomap_pub.cpp)
target_link_libraries(octomap_pub ${catkin_LIBRARIES})

add_executable(bender_planning src/bender_planning.cpp)
target_link_libraries(bender_planning ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(bender_planning bender_arm_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(tf_marker src/tf_marker.cpp)
target_link_libraries(tf_marker ${catkin_LIBRARIES})

add_executable(joint_states_remap src/joint_states_remap.cpp)
target_link_libraries(joint_states_remap ${catkin_LIBRARIES})

add_executable(arm_teleop src/arm_teleop.cpp)
target_link_libraries(arm_teleop ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#install(TARGETS move_group_interface_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(bender_arm_control_node bender_arm_control_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(bender_arm_control_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS bender_arm_control bender_arm_control_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_bender_arm_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
