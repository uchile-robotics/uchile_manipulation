cmake_minimum_required(VERSION 2.8.3)
project(bender_ikfast_arm)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  roscpp
  tf_conversions
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
    moveit_core
    pluginlib
    roscpp
    tf_conversions
)

include_directories(include)

set(IKFAST_LEFT_LIBRARY_NAME bender_l_arm_moveit_ikfast_plugin)
set(IKFAST_RIGHT_LIBRARY_NAME bender_r_arm_moveit_ikfast_plugin)

find_package(LAPACK REQUIRED)

add_library(${IKFAST_LEFT_LIBRARY_NAME} src/bender_l_arm_ikfast_moveit_plugin.cpp)
add_library(${IKFAST_RIGHT_LIBRARY_NAME} src/bender_r_arm_ikfast_moveit_plugin.cpp)
target_link_libraries(${IKFAST_LEFT_LIBRARY_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})
target_link_libraries(${IKFAST_RIGHT_LIBRARY_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})

install(TARGETS ${IKFAST_LEFT_LIBRARY_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(TARGETS ${IKFAST_RIGHT_LIBRARY_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
  FILES
  bender_l_arm_moveit_ikfast_plugin_description.xml
  bender_r_arm_moveit_ikfast_plugin_description.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)