cmake_minimum_required(VERSION 2.8.3)
project(fremenserver)


find_package(catkin REQUIRED COMPONENTS roscpp genmsg actionlib_msgs actionlib) 

set ( CMAKE_CXX_FLAGS "-Ofast -march=native -mno-avx -ggdb")

add_action_files(
  DIRECTORY action
  FILES Fremen.action
)

generate_messages(
   DEPENDENCIES actionlib_msgs 
)

catkin_package(CATKIN_DEPENDS roscpp message_runtime genmsg actionlib_msgs)

include_directories(${catkin_INCLUDE_DIRS}) 

add_executable(fremenserver src/fremenserver.cpp)

add_dependencies(fremenserver ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_library(fremenserver_frelement src/CFrelement.cpp)
add_library(fremenserver_timer src/CTimer.cpp)
add_library(fremenserver_frelementset src/CFrelementSet.cpp)

target_link_libraries(fremenserver_frelement fremenserver_timer)
target_link_libraries(fremenserver_frelementset fremenserver_frelement)
target_link_libraries (fremenserver ${catkin_LIBRARIES} fremenserver_frelementset)


# Mark executables and/or libraries for installation
install(TARGETS fremenserver fremenserver_frelement fremenserver_timer fremenserver_frelementset
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

