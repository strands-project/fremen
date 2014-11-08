cmake_minimum_required(VERSION 2.8.3)
project(frenap)


find_package(catkin REQUIRED COMPONENTS  mongodb_store roscpp genmsg actionlib_msgs actionlib strands_navigation_msgs std_msgs) 

set ( CMAKE_CXX_FLAGS "-Ofast -march=native -mno-avx -ggdb")

add_action_files(
  DIRECTORY action
  FILES TopologicalPrediction.action
)

generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

#catkin_package(CATKIN_DEPENDS roscpp message_runtime genmsg actionlib_msgs)
catkin_package()

include_directories(${catkin_INCLUDE_DIRS} include) 

add_executable(frenap src/frenap.cpp)
add_executable(frenap-standalone src/frenap-standalone.cpp)

add_dependencies(frenap ${catkin_EXPORTED_TARGETS} mongodb_store ${PROJECT_NAME}_generate_messages_cpp)

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

#############
## Install ##
#############
# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html
## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
#install(DIRECTORY scripts/
#DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#USE_SOURCE_PERMISSIONS
#)

# Mark other files for installation (e.g. launch and bag files, etc.)
#install(
#DIRECTORY launch
#DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

# Mark cpp header files for installation
install(DIRECTORY include/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS frenap frenap-standalone frenap_edge_statistics frenap_timer frenap_frelement
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )
