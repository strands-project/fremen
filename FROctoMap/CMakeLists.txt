cmake_minimum_required(VERSION 2.8.3)
project(fremen)


find_package(catkin REQUIRED COMPONENTS  pcl_conversions pcl_ros roscpp rospy std_msgs visualization_msgs geometry_msgs genmsg octomap_server sensor_msgs)
#set ( CMAKE_CXX_FLAGS "-O3 -march=native")
set ( CMAKE_CXX_FLAGS "-ggdb")

include_directories(${OCTOMAP_INCLUDE_DIRS}) 
link_libraries( ${OCTOMAP_LIBRARIES}) 

add_action_files(DIRECTORY action FILES froctomap.action)

add_service_files(
		FILES
		RecoverOctomap.srv
		EstimateOctomap.srv
		SaveGrid.srv
		UpdateGrid.srv
		EvaluateGrid.srv
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

add_library(frelement src/CFrelement.cpp)
add_library(fremen_timer src/CTimer.cpp)
add_library(fftplan src/CFFTPlan.cpp)
add_library(fremengrid src/CFremenGrid.cpp)

target_link_libraries(frelement fftw3)
target_link_libraries(frelement fftplan)
target_link_libraries(frelement fremen_timer)
target_link_libraries(fremengrid frelement)
target_link_libraries(froctomap fremengrid)
target_link_libraries(froctomap_util fremengrid)
target_link_libraries(convert_raw ${catkin_LIBRARIES})
target_link_libraries(froctomap ${catkin_LIBRARIES})
