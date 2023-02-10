cmake_minimum_required(VERSION 3.0.2)
project(image_transport_codecs)

find_package(catkin REQUIRED COMPONENTS
  class_loader
  compressed_depth_image_transport
  compressed_image_transport
  cras_cpp_common
  cras_topic_tools
  dynamic_reconfigure
  image_transport
  pluginlib
  sensor_msgs
  theora_image_transport
  topic_tools
)

#catkin_lint: ignore_once duplicate_find
find_package(compressed_image_transport QUIET)
if (compressed_image_transport_VERSION VERSION_GREATER_EQUAL "1.14.0")
  set(compressed_has_jpeg_options 1)
  set(compressed_depth_has_rvl 1)
else()
  set(compressed_has_jpeg_options 0)
  set(compressed_depth_has_rvl 0)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(TurboJPEG REQUIRED libturbojpeg)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} compressed_codec compressed_depth_codec
  CATKIN_DEPENDS class_loader cras_cpp_common dynamic_reconfigure image_transport pluginlib sensor_msgs topic_tools
)

include_directories(include ${catkin_INCLUDE_DIRS} ${TurboJPEG_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/image_transport_codec.cpp src/${PROJECT_NAME}.cpp src/parse_compressed_format.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_compile_definitions(${PROJECT_NAME} PRIVATE COMPRESSED_DEPTH_HAS_RVL=${compressed_depth_has_rvl})

add_library(compressed_codec src/codecs/compressed_codec.cpp)
target_link_libraries(compressed_codec ${catkin_LIBRARIES} ${TurboJPEG_LIBRARIES})
target_compile_definitions(compressed_codec PRIVATE COMPRESSED_HAS_JPEG_OPTIONS=${compressed_has_jpeg_options})

add_library(compressed_depth_codec src/codecs/compressed_depth_codec.cpp)
target_link_libraries(compressed_depth_codec ${catkin_LIBRARIES})
target_compile_definitions(compressed_depth_codec PRIVATE COMPRESSED_DEPTH_HAS_RVL=${compressed_depth_has_rvl})

add_library(raw_codec_plugin src/plugins/raw_codec_plugin.cpp)
target_link_libraries(raw_codec_plugin ${PROJECT_NAME} ${catkin_LIBRARIES})
class_loader_hide_library_symbols(raw_codec_plugin)

add_library(compressed_codec_plugin src/plugins/compressed_codec_plugin.cpp)
target_link_libraries(compressed_codec_plugin ${PROJECT_NAME} compressed_codec ${catkin_LIBRARIES})
class_loader_hide_library_symbols(compressed_codec_plugin)

add_library(compressed_depth_codec_plugin src/plugins/compressed_depth_codec_plugin.cpp)
target_link_libraries(compressed_depth_codec_plugin ${PROJECT_NAME} compressed_depth_codec ${catkin_LIBRARIES})
class_loader_hide_library_symbols(compressed_depth_codec_plugin)

install(TARGETS compressed_codec compressed_codec_plugin compressed_depth_codec compressed_depth_codec_plugin ${PROJECT_NAME} raw_codec_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES
  codecs.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  find_package(rosbag REQUIRED)

  # catkin_lint - checks validity of package.xml and CMakeLists.txt
  # ROS buildfarm calls this without any environment and with empty rosdep cache,
  # so we have problems reading the list of packages from env
  # see https://github.com/ros-infrastructure/ros_buildfarm/issues/923
  if(DEFINED ENV{ROS_HOME})
    #catkin_lint: ignore_once env_var
    set(ROS_HOME "$ENV{ROS_HOME}")
  else()
    #catkin_lint: ignore_once env_var
    set(ROS_HOME "$ENV{HOME}/.ros")
  endif()

  #catkin_lint: ignore_once env_var
  if(DEFINED ENV{ROS_ROOT} AND EXISTS "${ROS_HOME}/rosdep/sources.cache")
    roslint_custom(catkin_lint "-W2" .)
  endif()

  # Roslint C++ - checks formatting and some other rules for C++ files

  file(GLOB_RECURSE ROSLINT_INCLUDE include/*.h include/*.hpp)
  file(GLOB_RECURSE ROSLINT_SRC src/*.cpp src/*.hpp src/*.h)
  file(GLOB_RECURSE ROSLINT_TEST test/*.cpp)

  set(ROSLINT_CPP_OPTS "--extensions=h,hpp,hh,c,cpp,cc;--linelength=120;--filter=\
    -build/header_guard,-readability/namespace,-whitespace/braces,-runtime/references,\
    -build/c++11,-readability/nolint,-readability/todo,-legal/copyright")
  roslint_cpp(${ROSLINT_INCLUDE} ${ROSLINT_SRC})

  set(ROSLINT_CPP_OPTS "${ROSLINT_CPP_OPTS},-build/namespaces")  # allow "using namespace" in tests
  roslint_cpp(${ROSLINT_TEST})

  roslint_add_test()

  include_directories(${rosbag_INCLUDE_DIRS})
  
  catkin_add_gtest(test_codecs test/test_codecs.cpp)
  target_link_libraries(test_codecs ${PROJECT_NAME} ${catkin_LIBRARIES} ${rosbag_LIBRARIES})
  add_dependencies(test_codecs raw_codec_plugin compressed_codec_plugin compressed_depth_codec_plugin)
  target_compile_definitions(test_codecs PRIVATE COMPRESSED_HAS_JPEG_OPTIONS=${compressed_has_jpeg_options})
  target_compile_definitions(test_codecs PRIVATE COMPRESSED_DEPTH_HAS_RVL=${compressed_depth_has_rvl})
  target_compile_definitions(test_codecs PRIVATE TEST_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/data\")

  catkin_add_gtest(test_parse_compressed_format test/test_parse_compressed_format.cpp)
  target_link_libraries(test_parse_compressed_format ${PROJECT_NAME} ${rosbag_LIBRARIES})
  target_compile_definitions(test_parse_compressed_format PRIVATE COMPRESSED_DEPTH_HAS_RVL=${compressed_depth_has_rvl})
  target_compile_definitions(test_parse_compressed_format PRIVATE TEST_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/data\")
  add_dependencies(test_parse_compressed_format ${catkin_EXPORTED_TARGETS})

  catkin_add_nosetests(test/test_codecs.py DEPENDENCIES compressed_codec_plugin compressed_depth_codec_plugin ${PROJECT_NAME} raw_codec_plugin ${catkin_EXPORTED_TARGETS})
  catkin_add_nosetests(test/test_parse_compressed_format.py DEPENDENCIES ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
endif()
