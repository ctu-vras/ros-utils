cmake_minimum_required(VERSION 2.8.3)
project(robot_mapping)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
  message_filters
)

find_package( PCL REQUIRED )
find_package( Eigen3 REQUIRED )
find_package( ompl REQUIRED )
find_package( Boost )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot
#  CATKIN_DEPENDS nav_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
##include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${Boost_LIBRARIES}
  -lboost_system
)

# include folder with libraries
link_directories(libs)

## Declare a C++ executable
add_executable(mapping_node src/main_node.cpp src/mapping.cpp src/pose6DOF.cpp src/map25Dplanner.cpp src/commander.cpp )


 target_link_libraries(mapping_node
   libquadLib.a
   libcloudOPLib.a
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${EIGEN3_LIBS}
   ${OMPL_LIBRARIES}
   ${Boost_LIBRARIES}
   -lboost_system
 )


