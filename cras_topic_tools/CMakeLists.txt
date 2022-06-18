cmake_minimum_required(VERSION 3.7.2)
project(cras_topic_tools)

find_package(catkin REQUIRED COMPONENTS
  cras_cpp_common
  nodelet
  roscpp
  topic_tools
)

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cras_shape_shifter
)

include_directories(include ${catkin_INCLUDE_DIRS})

set(INCLUDE_BASE "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}")

add_library(cras_shape_shifter src/shape_shifter.cpp)
add_dependencies(cras_shape_shifter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cras_shape_shifter ${catkin_LIBRARIES})

add_library(cras_lazy_subscriber INTERFACE)
target_sources(cras_lazy_subscriber INTERFACE $<BUILD_INTERFACE:${INCLUDE_BASE}/lazy_subscriber.hpp>)
target_link_libraries(cras_lazy_subscriber INTERFACE ${catkin_LIBRARIES} Boost::boost)

add_library(cras_generic_lazy_pubsub INTERFACE)
target_sources(cras_generic_lazy_pubsub INTERFACE $<BUILD_INTERFACE:${INCLUDE_BASE}/generic_lazy_pubsub.hpp>)
target_link_libraries(cras_generic_lazy_pubsub INTERFACE ${catkin_LIBRARIES} Boost::boost)

add_library(cras_count_messages_nodelet src/count_messages.cpp)
add_dependencies(cras_count_messages_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cras_count_messages_nodelet ${catkin_LIBRARIES})
cras_node_from_nodelet(cras_count_messages_nodelet ${PROJECT_NAME}/count_messages.h cras::CountMessagesNodelet OUTPUT_NAME count_messages)

add_library(cras_throttle_messages_nodelet src/throttle_messages.cpp)
add_dependencies(cras_throttle_messages_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cras_throttle_messages_nodelet ${catkin_LIBRARIES})
cras_node_from_nodelet(cras_throttle_messages_nodelet ${PROJECT_NAME}/throttle_messages.h cras::ThrottleMessagesNodelet OUTPUT_NAME throttle ANONYMOUS)

add_library(cras_relay_nodelet src/relay.cpp)
add_dependencies(cras_relay_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cras_relay_nodelet ${catkin_LIBRARIES})
cras_node_from_nodelet(cras_relay_nodelet ${PROJECT_NAME}/relay.h cras::RelayNodelet OUTPUT_NAME relay ANONYMOUS)

add_library(cras_repeat_nodelet src/repeat.cpp)
add_dependencies(cras_repeat_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cras_repeat_nodelet cras_shape_shifter ${catkin_LIBRARIES})
cras_node_from_nodelet(cras_repeat_nodelet ${PROJECT_NAME}/repeat.h cras::RepeatMessagesNodelet OUTPUT_NAME repeat ANONYMOUS)

add_library(cras_change_header_nodelet src/change_header.cpp)
add_dependencies(cras_change_header_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cras_change_header_nodelet cras_shape_shifter ${catkin_LIBRARIES})
cras_node_from_nodelet(cras_change_header_nodelet ${PROJECT_NAME}/change_header.h cras::ChangeHeaderNodelet OUTPUT_NAME change_header ANONYMOUS)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING REGEX ".*\\.hp?p?$"
)

install(TARGETS cras_count_messages_nodelet cras_throttle_messages_nodelet cras_relay_nodelet cras_shape_shifter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES
  nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS nodes/filter nodes/mux_replay
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(geometry_msgs REQUIRED)

  catkin_add_gtest(test_shape_shifter test/test_shape_shifter.cpp)
  target_link_libraries(test_shape_shifter cras_shape_shifter ${catkin_LIBRARIES} ${geometry_msgs_LIBRARIES})

  add_rostest_gtest(test_cras_lazy_subscriber test/test_lazy_subscriber.test test/test_lazy_subscriber.cpp)
  target_link_libraries(test_cras_lazy_subscriber cras_lazy_subscriber ${catkin_LIBRARIES})

  add_rostest(test/test_count_messages.test DEPENDENCIES cras_count_messages_nodelet cras_count_messages_nodelet_node)
  
  add_rostest_gtest(test_cras_generic_lazy_pubsub test/test_generic_lazy_pubsub.test test/test_generic_lazy_pubsub.cpp)
  target_link_libraries(test_cras_generic_lazy_pubsub cras_generic_lazy_pubsub ${catkin_LIBRARIES})
  add_dependencies(test_cras_generic_lazy_pubsub cras_count_messages_nodelet_node)
  
  add_rostest(test/test_relay.test DEPENDENCIES cras_relay_nodelet_node)
  add_rostest(test/test_relay_nodelet.test DEPENDENCIES cras_relay_nodelet)
  
  add_rostest(test/test_throttle.test DEPENDENCIES cras_throttle_messages_nodelet_node)
  add_rostest(test/test_throttle_nodelet.test DEPENDENCIES cras_throttle_messages_nodelet)
  
  add_rostest(test/test_repeat.test DEPENDENCIES cras_repeat_nodelet_node)
  add_rostest(test/test_repeat_nodelet.test DEPENDENCIES cras_repeat_nodelet)
  
  add_rostest(test/test_change_header.test DEPENDENCIES cras_change_header_nodelet_node)
  add_rostest(test/test_change_header_nodelet.test DEPENDENCIES cras_change_header_nodelet)
  
  add_rostest(test/test_mux_replay.test)
  add_rostest(test/test_mux_replay_loop.test)
  
  add_rostest(test/test_filter.test)
endif()
