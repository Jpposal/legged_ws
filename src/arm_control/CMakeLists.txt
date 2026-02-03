cmake_minimum_required(VERSION 3.0.2)
project(arm_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Must set BOOST_MPL limits BEFORE any headers are included
add_definitions(-DBOOST_MPL_LIMIT_LIST_SIZE=30)
add_definitions(-DBOOST_MPL_CFG_NO_PREPROCESSED_HEADERS)
add_definitions(-DPINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_interface
  hardware_interface
  pluginlib
  geometry_msgs
  gazebo_msgs
  my_robot_controllers
)

find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PkgConfig REQUIRED)

pkg_check_modules(pinocchio REQUIRED pinocchio)
pkg_check_modules(hpp_fcl REQUIRED hpp-fcl)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES arm_controller_lib
  CATKIN_DEPENDS roscpp controller_interface hardware_interface pluginlib geometry_msgs gazebo_msgs
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
  ${hpp_fcl_INCLUDE_DIRS}
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
  ${hpp_fcl_LIBRARY_DIRS}
)

# Controller Library
add_library(arm_controller_lib
  src/ArmController.cpp
  src/ArmController_Leader.cpp
  src/ArmController_Follower.cpp
)
target_link_libraries(arm_controller_lib
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${pinocchio_LIBRARIES}
  ${hpp_fcl_LIBRARIES}
)

install(TARGETS arm_controller_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES arm_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
add_executable(traj_publisher_node src/traj_publisher_node.cpp)
target_link_libraries(traj_publisher_node ${catkin_LIBRARIES})
