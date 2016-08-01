cmake_minimum_required(VERSION 2.8.3)
project(bender_arm_planning)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  cmake_modules
  message_generation
  moveit_core
  moveit_ros_move_group
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_simple_grasps
  moveit_visual_tools
  pluginlib
  roscpp
  std_msgs
  std_srvs
  tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED serialization system filesystem signals thread)
find_package(Eigen REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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
add_message_files(
  FILES
  OctomapOptions.msg
  GraspPosition.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AddObjectACM.srv
  CapabilityMapBestDistance.srv
  CapabilityMapGrasp.srv
  ManageOctomap.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  trajectory_msgs
  moveit_msgs
)

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
  INCLUDE_DIRS
  	include
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_move_group
    message_runtime
#  DEPENDS system_lib
  LIBRARIES
  	bender_move_group_capabilities
    capability_map
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_library(capability_map
  src/capability_map.cpp
  src/grasp_point.cpp
)
add_dependencies(capability_map ${catkin_EXPORTED_TARGETS})
target_link_libraries(capability_map ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(bender_move_group_capabilities 
  src/manage_octomap_service_capability.cpp
  src/capability_map_service.cpp
)
add_dependencies(bender_move_group_capabilities ${catkin_EXPORTED_TARGETS})
target_link_libraries(bender_move_group_capabilities capability_map ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(capmap_service src/capmap_service.cpp)
target_link_libraries(capmap_service capability_map ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(capability_map_gen src/capability_map_generator.cpp)
target_link_libraries(capability_map_gen capability_map ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(cm_load_test src/cm_load_test.cpp)
target_link_libraries(cm_load_test capability_map ${catkin_LIBRARIES} ${Boost_LIBRARIES})

## Declare a cpp library
# add_library(bender_arm_planning
#   src/${PROJECT_NAME}/bender_arm_planning.cpp
# )

## Declare a cpp executable
# add_executable(bender_arm_planning_node src/bender_arm_planning_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(bender_arm_planning_node bender_arm_planning_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(bender_arm_planning_node
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
# install(TARGETS bender_arm_planning bender_arm_planning_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_bender_arm_planning.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
