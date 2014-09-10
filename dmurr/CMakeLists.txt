cmake_minimum_required(VERSION 2.8.3)
project(dmurr)


#MIGRATE find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg sensor_msgs mongodb_store_cpp_client actionlib_msgs actionlib) 
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg sensor_msgs ros_datacentre_cpp_client actionlib_msgs actionlib)

#set ( CMAKE_CXX_FLAGS "-O3 -march=native")
set ( CMAKE_CXX_FLAGS "-ggdb -march=native -std=c++0x")

add_action_files(
  DIRECTORY action
  FILES TopologicalPrediction.action
)

generate_messages(
   DEPENDENCIES actionlib_msgs 
)


catkin_package(CATKIN_DEPENDS roscpp message_runtime std_msgs genmsg actionlib_msgs)

include_directories(${catkin_INCLUDE_DIRS}) 


add_executable(dmurr src/dmurr.cpp)
add_executable(dmurr-standalone src/dmurr-standalone.cpp)

add_dependencies(dmurr ${catkin_EXPORTED_TARGETS} ros_datacentre)

add_library(dmurr_frelement src/CFrelement.cpp)
add_library(dmurr_edge_statistics src/CEdgeStatistics.cpp)
add_library(dmurr_timer src/CTimer.cpp)

target_link_libraries(dmurr_frelement dmurr_timer)
target_link_libraries(dmurr dmurr_edge_statistics)
target_link_libraries(dmurr-standalone dmurr_frelement)

target_link_libraries (dmurr
		${catkin_LIBRARIES}
		mongoclient
		message_store
		dmurr_frelement
)
