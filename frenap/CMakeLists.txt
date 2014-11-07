cmake_minimum_required(VERSION 2.8.3)
project(frenap)


find_package(catkin REQUIRED COMPONENTS  mongodb_store roscpp genmsg actionlib_msgs actionlib strands_navigation_msgs) 

set ( CMAKE_CXX_FLAGS "-Ofast -march=native -mno-avx -ggdb")

add_action_files(
  DIRECTORY action
  FILES TopologicalPrediction.action
)

generate_messages(
   DEPENDENCIES actionlib_msgs 
)

#add_dependencies(frenap ${catkin_EXPORTED_TARGETS})
catkin_package(CATKIN_DEPENDS roscpp message_runtime genmsg actionlib_msgs)

include_directories(${catkin_INCLUDE_DIRS}) 

add_executable(frenap src/frenap.cpp)
add_executable(frenap-standalone src/frenap-standalone.cpp)

add_dependencies(frenap ${catkin_EXPORTED_TARGETS} mongodb_store topological_navigation_generate_messages_cpp actionlib_generate_messages_cpp)

add_library(frenap_frelement src/CFrelement.cpp)
add_library(frenap_edge_statistics src/CEdgeStatistics.cpp)
add_library(frenap_timer src/CTimer.cpp)

target_link_libraries(frenap_frelement frenap_timer)
target_link_libraries(frenap frenap_edge_statistics)
target_link_libraries(frenap-standalone frenap_frelement)

target_link_libraries (frenap
		${catkin_LIBRARIES}
		frenap_frelement
)
