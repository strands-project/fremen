cmake_minimum_required(VERSION 2.8.3)
project(fremen2dgrid)


find_package(catkin REQUIRED COMPONENTS roscpp genmsg actionlib_msgs actionlib nav_msgs) 

set ( CMAKE_CXX_FLAGS "-Ofast -march=native -mno-avx -ggdb")

add_action_files(
  DIRECTORY action
  FILES Fremen2DGrid.action
)

generate_messages(
   DEPENDENCIES actionlib_msgs nav_msgs 
)

#catkin_package(CATKIN_DEPENDS roscpp message_runtime genmsg actionlib_msgs nav_msgs)
catkin_package()

include_directories(${catkin_INCLUDE_DIRS}) 

add_executable(fremen2DGrid src/fremen2DGrid.cpp)

add_dependencies(fremen2DGrid ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_library(fremen2DGrid_frelement src/CFrelement.cpp)
add_library(fremen2DGrid_grid src/CFrelement2DGrid.cpp)
add_library(fremen2DGrid_set src/CFrelement2DGridSet.cpp)

target_link_libraries (fremen2DGrid_grid fremen2DGrid_frelement)
target_link_libraries (fremen2DGrid_set fremen2DGrid_grid)
target_link_libraries (fremen2DGrid ${catkin_LIBRARIES} fremen2DGrid_set)

# Mark executables and/or libraries for installation
install(TARGETS fremen2DGrid fremen2DGrid_frelement fremen2DGrid_grid fremen2DGrid_set
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

