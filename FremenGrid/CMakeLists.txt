cmake_minimum_required(VERSION 2.8.3)
project(fremengrid)


find_package(catkin REQUIRED COMPONENTS  pcl_conversions pcl_ros roscpp rospy std_msgs visualization_msgs geometry_msgs genmsg sensor_msgs image_transport)
set ( CMAKE_CXX_FLAGS "-Ofast -march=native -mno-avx -ggdb")

include_directories(${OCTOMAP_INCLUDE_DIRS}) 
link_libraries( ${OCTOMAP_LIBRARIES}) 

add_service_files(
		FILES
		SaveLoad.srv
		AddView.srv
		Visualize.srv
		Entropy.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs actionlib_msgs
)

catkin_package(CATKIN_DEPENDS roscpp message_runtime pcl_conversions pcl_ros std_msgs genmsg actionlib_msgs)

include_directories(${catkin_INCLUDE_DIRS}) 


add_executable(fremengrid src/fremenGrid.cpp)
add_executable(exploration src/fremenExploration.cpp)

add_library(frelement src/CFrelement.cpp)
add_library(fremen_timer src/CTimer.cpp)
add_library(fremengridlib src/CFremenGrid.cpp)

target_link_libraries(frelement fremen_timer)
target_link_libraries(fremengridlib frelement)

target_link_libraries(fremengrid fremengridlib)
target_link_libraries(fremengrid ${catkin_LIBRARIES})

target_link_libraries(exploration fremengridlib)
target_link_libraries(exploration ${catkin_LIBRARIES})
