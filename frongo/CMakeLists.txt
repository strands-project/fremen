cmake_minimum_required(VERSION 2.8.3)
project(frongo)

find_package(catkin REQUIRED COMPONENTS fremenserver mongodb_store)

catkin_python_setup()

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
