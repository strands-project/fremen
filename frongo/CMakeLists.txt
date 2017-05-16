cmake_minimum_required(VERSION 2.8.3)
project(frongo)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs message_generation fremenserver mongodb_store)

catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

add_service_files(
  FILES
  AddModel.srv
  BuildModel.srv
  DetectAnomalies.srv
  GraphModel.srv
  GetInfo.srv
  PredictState.srv
  PredictStateOrder.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs 
)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############

 install(PROGRAMS
   scripts/fremeniser.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
