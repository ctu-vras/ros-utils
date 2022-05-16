cmake_minimum_required(VERSION 3.0.2)
project(cras_topic_tools)

find_package(catkin REQUIRED COMPONENTS
  cras_cpp_common
  nodelet
  roscpp
  topic_tools
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(cras_throttle_messages_nodelet src/throttle_messages.cpp)
add_dependencies(cras_throttle_messages_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cras_throttle_messages_nodelet ${catkin_LIBRARIES})
cras_node_from_nodelet(cras_throttle_messages_nodelet ${PROJECT_NAME}/throttle_messages.h cras::ThrottleMessagesNodelet throttle_messages)

add_library(cras_relay_nodelet src/relay.cpp)
add_dependencies(cras_relay_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cras_relay_nodelet ${catkin_LIBRARIES})
cras_node_from_nodelet(cras_relay_nodelet ${PROJECT_NAME}/relay.h cras::RelayNodelet relay)

add_executable(repeater src/repeater.cpp)
target_link_libraries(repeater ${catkin_LIBRARIES})
add_dependencies(repeater ${catkin_EXPORTED_TARGETS})

add_executable(joy_repeater src/joy_repeater.cpp)
target_link_libraries(joy_repeater ${catkin_LIBRARIES})
add_dependencies(joy_repeater ${catkin_EXPORTED_TARGETS})

install(TARGETS cras_throttle_messages_nodelet cras_relay_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES
  nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS nodes/filter nodes/repeat
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
