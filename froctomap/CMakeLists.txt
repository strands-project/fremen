cmake_minimum_required(VERSION 2.8.3)
project(froctomap)


find_package(catkin REQUIRED COMPONENTS  pcl_conversions pcl_ros roscpp rospy std_msgs visualization_msgs geometry_msgs genmsg octomap_server sensor_msgs)
#set ( CMAKE_CXX_FLAGS "-O3 -march=native")
set ( CMAKE_CXX_FLAGS "-ggdb")

include_directories(${OCTOMAP_INCLUDE_DIRS}) 
link_libraries( ${OCTOMAP_LIBRARIES}) 

add_action_files(DIRECTORY action FILES froctomap.action)

add_service_files(
		FILES
		Evaluate.srv
		Estimate.srv
		Recover.srv
		Retrieve.srv
		SaveLoad.srv
		Update.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs actionlib_msgs
)

catkin_package(CATKIN_DEPENDS roscpp message_runtime pcl_conversions pcl_ros std_msgs genmsg actionlib_msgs)

include_directories(${catkin_INCLUDE_DIRS}) 


add_executable(froctomap src/froctomap.cpp)
add_executable(convert_raw src/convert_raw.cpp)
add_executable(froctomap_util src/froctomap_util.cpp)

add_library(froctomap_element src/CFrelement.cpp)
add_library(froctomap_timer src/CTimer.cpp)
add_library(froctomap_fftplan src/CFFTPlan.cpp)
add_library(froctomap_grid src/CFremenGrid.cpp)

target_link_libraries(froctomap_element fftw3)
target_link_libraries(froctomap_element froctomap_fftplan)
target_link_libraries(froctomap_element froctomap_timer)
target_link_libraries(froctomap_grid froctomap_element)
target_link_libraries(froctomap froctomap_grid)
target_link_libraries(froctomap_util froctomap_grid)
target_link_libraries(convert_raw ${catkin_LIBRARIES})
target_link_libraries(froctomap ${catkin_LIBRARIES})
