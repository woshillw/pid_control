cmake_minimum_required(VERSION 2.8.3)
project(pid_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
)

catkin_package(
  INCLUDE_DIRS include ${Eigen3_INCLUDE_DIRS}
  LIBRARIES llwpid 
  CATKIN_DEPENDS roscpp 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(llwpid
  src/llwpid.cpp
)

install(TARGETS llwpid 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)