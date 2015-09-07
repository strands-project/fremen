cmake_minimum_required(VERSION 2.8.3)
project(fremenarray)


find_package(catkin REQUIRED COMPONENTS roscpp genmsg actionlib_msgs actionlib) 

set ( CMAKE_CXX_FLAGS "-Ofast -march=native -mno-avx -ggdb")

add_action_files(
  DIRECTORY action
  FILES FremenArray.action
)

generate_messages(
   DEPENDENCIES actionlib_msgs 
)

catkin_package(CATKIN_DEPENDS roscpp message_runtime genmsg actionlib_msgs)

include_directories(${catkin_INCLUDE_DIRS}) 

add_executable(fremenarray src/fremenarray.cpp)

add_dependencies(fremenarray ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_library(fremenarray_frelement src/CFrelement.cpp)

target_link_libraries (fremenarray ${catkin_LIBRARIES} fremenarray_frelement)

# Mark executables and/or libraries for installation
install(TARGETS fremenarray fremenarray_frelement
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

