cmake_minimum_required(VERSION 3.0.2)
project(camera_throttle)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  cras_cpp_common
  cv_bridge
  image_transport
  nodelet
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} rgbd_throttle rgbd_image_transport
  CATKIN_DEPENDS cras_cpp_common image_transport nodelet roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/camera_throttle.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_library(rgbd_image_transport src/rgbd_camera_subscriber.cpp src/rgbd_image_transport.cpp src/rgbd_camera_publisher.cpp)
add_dependencies(rgbd_image_transport ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rgbd_image_transport ${catkin_LIBRARIES})

add_library(rgbd_throttle src/rgbd_throttle.cpp)
add_dependencies(rgbd_throttle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rgbd_throttle ${catkin_LIBRARIES} rgbd_image_transport)

install(TARGETS ${PROJECT_NAME} rgbd_throttle
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES
  nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)