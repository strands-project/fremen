cmake_minimum_required(VERSION 2.8.3)
project(fremengrid)


find_package(catkin REQUIRED COMPONENTS  roscpp rospy std_msgs visualization_msgs geometry_msgs genmsg sensor_msgs image_transport actionlib actionlib_msgs tf  move_base_msgs)
set ( CMAKE_CXX_FLAGS "-Ofast -march=native -mno-avx -ggdb")

add_service_files(
		FILES
		SaveLoad.srv
		AddView.srv
		Visualize.srv
		Entropy.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs 
	actionlib_msgs
)

include_directories(${catkin_INCLUDE_DIRS} include) 

add_executable(fremengrid src/fremenGrid.cpp)
add_executable(exploration src/fremenExploration.cpp)

add_dependencies(fremengrid ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(exploration ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

#catkin_package(CATKIN_DEPENDS roscpp message_runtime genmsg actionlib_msgs)
catkin_package()


add_library(frelement src/CFrelement.cpp)
add_library(fremen_timer src/CTimer.cpp)
add_library(fremengridlib src/CFremenGrid.cpp)

target_link_libraries(frelement fremen_timer)
target_link_libraries(fremengridlib frelement)

target_link_libraries(fremengrid fremengridlib)
target_link_libraries(fremengrid ${catkin_LIBRARIES})

target_link_libraries(exploration fremengridlib)
target_link_libraries(exploration ${catkin_LIBRARIES})

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
#install(
#		DIRECTORY launch
#		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#       )
# Mark cpp header files for installation
	install(DIRECTORY include/
			DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	       )
## Mark executables and/or libraries for installation
install(TARGETS exploration fremengrid frelement fremengridlib fremen_timer frelement 
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )
