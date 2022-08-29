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

add_library(cras_priority_mux_nodelet src/priority_mux.cpp)
add_dependencies(cras_priority_mux_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cras_priority_mux_nodelet ${catkin_LIBRARIES})
cras_node_from_nodelet(cras_priority_mux_nodelet ${PROJECT_NAME}/priority_mux.h cras::PriorityMuxNodelet OUTPUT_NAME priority_mux ANONYMOUS)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING REGEX ".*\\.hp?p?$"
)

install(TARGETS
  cras_change_header_nodelet cras_count_messages_nodelet cras_priority_mux_nodelet cras_relay_nodelet
  cras_repeat_nodelet cras_shape_shifter cras_throttle_messages_nodelet
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
  find_package(geometry_msgs REQUIRED)
  find_package(roslaunch REQUIRED)
  find_package(roslint REQUIRED)
  find_package(rostest REQUIRED)

  # catkin_lint - checks validity of package.xml and CMakeLists.txt

  roslint_custom(catkin_lint "-W2" .)

  # Roslint C++ - checks formatting and some other rules for C++ files

  file(GLOB_RECURSE ROSLINT_INCLUDE include/*.h include/*.hpp)
  file(GLOB_RECURSE ROSLINT_SRC src/*.cpp src/*.hpp src/*.h)
  file(GLOB_RECURSE ROSLINT_TEST test/*.cpp)

  set(ROSLINT_CPP_OPTS "--extensions=h,hpp,hh,c,cpp,cc;--linelength=120;--filter=\
    -build/header_guard,-readability/namespace,-whitespace/braces,-runtime/references,\
    -build/c++11,-readability/nolint,-readability/todo,-legal/copyright")
  roslint_cpp(${ROSLINT_INCLUDE} ${ROSLINT_SRC})

  set(ROSLINT_CPP_OPTS "${ROSLINT_CPP_OPTS},-build/namespaces")  # allow "using namespace cras;" in tests
  roslint_cpp(${ROSLINT_TEST})

  add_dependencies(run_tests roslint_${PROJECT_NAME})

  # GTest tests

  catkin_add_gtest(test_shape_shifter test/test_shape_shifter.cpp)
  target_link_libraries(test_shape_shifter cras_shape_shifter ${catkin_LIBRARIES} ${geometry_msgs_LIBRARIES})

  add_rostest_gtest(test_cras_lazy_subscriber test/test_lazy_subscriber.test test/test_lazy_subscriber.cpp)
  target_link_libraries(test_cras_lazy_subscriber cras_lazy_subscriber ${catkin_LIBRARIES})
  roslaunch_add_file_check(test/test_lazy_subscriber.test USE_TEST_DEPENDENCIES)

  add_rostest(test/test_count_messages.test DEPENDENCIES cras_count_messages_nodelet cras_count_messages_nodelet_node)
  roslaunch_add_file_check(test/test_count_messages.test USE_TEST_DEPENDENCIES)

  add_rostest_gtest(test_cras_generic_lazy_pubsub test/test_generic_lazy_pubsub.test test/test_generic_lazy_pubsub.cpp)
  target_link_libraries(test_cras_generic_lazy_pubsub cras_generic_lazy_pubsub ${catkin_LIBRARIES})
  add_dependencies(test_cras_generic_lazy_pubsub cras_count_messages_nodelet_node)
  roslaunch_add_file_check(test/test_generic_lazy_pubsub.test USE_TEST_DEPENDENCIES)

  add_rostest(test/test_relay.test DEPENDENCIES cras_relay_nodelet_node)
  roslaunch_add_file_check(test/test_relay.test USE_TEST_DEPENDENCIES)
  add_rostest(test/test_relay_nodelet.test DEPENDENCIES cras_relay_nodelet)
  roslaunch_add_file_check(test/test_relay_nodelet.test USE_TEST_DEPENDENCIES)

  add_rostest(test/test_throttle.test DEPENDENCIES cras_throttle_messages_nodelet_node)
  roslaunch_add_file_check(test/test_throttle.test USE_TEST_DEPENDENCIES)
  add_rostest(test/test_throttle_nodelet.test DEPENDENCIES cras_throttle_messages_nodelet)
  roslaunch_add_file_check(test/test_throttle_nodelet.test USE_TEST_DEPENDENCIES)

  add_rostest(test/test_repeat.test DEPENDENCIES cras_repeat_nodelet_node)
  roslaunch_add_file_check(test/test_repeat.test USE_TEST_DEPENDENCIES)
  add_rostest(test/test_repeat_nodelet.test DEPENDENCIES cras_repeat_nodelet)
  roslaunch_add_file_check(test/test_repeat_nodelet.test USE_TEST_DEPENDENCIES)

  add_rostest(test/test_change_header.test DEPENDENCIES cras_change_header_nodelet_node)
  roslaunch_add_file_check(test/test_change_header.test USE_TEST_DEPENDENCIES)
  add_rostest(test/test_change_header_nodelet.test DEPENDENCIES cras_change_header_nodelet)
  roslaunch_add_file_check(test/test_change_header_nodelet.test USE_TEST_DEPENDENCIES)

  add_rostest(test/test_mux_replay.test)
  roslaunch_add_file_check(test/test_mux_replay.test USE_TEST_DEPENDENCIES)
  add_rostest(test/test_mux_replay_loop.test)
  roslaunch_add_file_check(test/test_mux_replay_loop.test USE_TEST_DEPENDENCIES)

  add_rostest(test/test_filter.test)
  roslaunch_add_file_check(test/test_filter.test USE_TEST_DEPENDENCIES)
endif()
