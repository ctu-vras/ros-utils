cmake_minimum_required(VERSION 3.7.2)
project(cras_topic_tools)

find_package(catkin REQUIRED COMPONENTS
  cras_cpp_common
  nodelet
  roscpp
  topic_tools
)

find_package(Boost REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

set(INCLUDE_BASE "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}")

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
cras_node_from_nodelet(cras_throttle_messages_nodelet ${PROJECT_NAME}/throttle_messages.h cras::ThrottleMessagesNodelet OUTPUT_NAME throttle_messages)

add_library(cras_relay_nodelet src/relay.cpp)
add_dependencies(cras_relay_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cras_relay_nodelet ${catkin_LIBRARIES})
cras_node_from_nodelet(cras_relay_nodelet ${PROJECT_NAME}/relay.h cras::RelayNodelet OUTPUT_NAME relay ANONYMOUS)

add_executable(repeater src/repeater.cpp)
target_link_libraries(repeater ${catkin_LIBRARIES})
add_dependencies(repeater ${catkin_EXPORTED_TARGETS})

add_executable(joy_repeater src/joy_repeater.cpp)
target_link_libraries(joy_repeater ${catkin_LIBRARIES})
add_dependencies(joy_repeater ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING REGEX ".*\\.hp?p?$"
)

install(TARGETS cras_count_messages_nodelet cras_throttle_messages_nodelet cras_relay_nodelet
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

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(test_cras_lazy_subscriber test/test_lazy_subscriber.test test/test_lazy_subscriber.cpp)
  target_link_libraries(test_cras_lazy_subscriber cras_lazy_subscriber ${catkin_LIBRARIES})

  add_rostest_gtest(test_cras_generic_lazy_pubsub test/test_generic_lazy_pubsub.test test/test_generic_lazy_pubsub.cpp)
  target_link_libraries(test_cras_generic_lazy_pubsub cras_generic_lazy_pubsub ${catkin_LIBRARIES})
  add_dependencies(test_cras_generic_lazy_pubsub cras_count_messages_nodelet_node)
  
  add_rostest(test/test_relay.test DEPENDENCIES cras_relay_nodelet_node)
  add_rostest(test/test_relay_nodelet.test DEPENDENCIES cras_relay_nodelet)
endif()
