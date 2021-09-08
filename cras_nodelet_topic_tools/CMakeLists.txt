cmake_minimum_required(VERSION 3.0.2)
project(cras_nodelet_topic_tools)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  topic_tools
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_library(cras_throttle_nodelet src/throttle.cpp)
add_dependencies(cras_throttle_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cras_throttle_nodelet ${catkin_LIBRARIES})

add_library(cras_relay_nodelet src/relay.cpp)
add_dependencies(cras_relay_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cras_relay_nodelet ${catkin_LIBRARIES})

install(TARGETS cras_throttle_nodelet cras_relay_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES
  nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)