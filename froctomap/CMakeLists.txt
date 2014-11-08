cmake_minimum_required(VERSION 2.8.3)
project(froctomap)


find_package(catkin REQUIRED COMPONENTS  pcl_conversions pcl_ros roscpp rospy std_msgs visualization_msgs geometry_msgs genmsg octomap_server sensor_msgs)
#set ( CMAKE_CXX_FLAGS "-O3 -march=native")
set ( CMAKE_CXX_FLAGS "-ggdb")



include_directories(${OCTOMAP_INCLUDE_DIRS}) 
link_libraries( ${OCTOMAP_LIBRARIES}) 
#include_directories(${FFTW_INCLUDE_DIRS}) 
#link_libraries(${FFTW_LIBRARIES}) 

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

include_directories(${catkin_INCLUDE_DIRS} include) 

generate_messages(
	DEPENDENCIES
	std_msgs 
	actionlib_msgs
)

#catkin_package(CATKIN_DEPENDS roscpp message_runtime pcl_conversions pcl_ros std_msgs genmsg actionlib_msgs)
catkin_package()

add_executable(froctomap src/froctomap.cpp)
add_executable(convert_raw src/convert_raw.cpp)
add_executable(froctomap_util src/froctomap_util.cpp)

add_dependencies(froctomap ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

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

#############
## Install ##
#############
# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html
## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
	#install(DIRECTORY scripts/
	#		DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	#		USE_SOURCE_PERMISSIONS
	#       )


# Mark other files for installation (e.g. launch and bag files, etc.)
	install(
			DIRECTORY launch
			DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	       )


# Mark cpp header files for installation
	install(DIRECTORY include/
			DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	       )


## Mark executables and/or libraries for installation
install(TARGETS froctomap convert_raw froctomap_util froctomap_element froctomap_timer froctomap_fftplan froctomap_grid 
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

