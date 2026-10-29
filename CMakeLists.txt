cmake_minimum_required(VERSION 3.0.2)
project(autonomous_robot)
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} -O3 -Wall")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  map_manager
  frontiers_manager
  global_planner
  communication_manager
)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs nav_msgs geometry_msgs tf2 tf2_geometry_msgs map_manager frontiers_manager global_planner communication_manager
)
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(sa_exploration_node src/sa_exploration_node.cpp src/SingleAgentExploration.cpp src/spotPlatform.cpp)
add_dependencies(sa_exploration_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(sa_exploration_node ${catkin_LIBRARIES})
install(TARGETS sa_exploration_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch cfg DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
