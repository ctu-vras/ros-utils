cmake_minimum_required(VERSION 3.20.2)
project(cras_bag_tools)

find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_ros REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/extract_images
#  scripts/filter_bag
#  scripts/fix_msg_defs
#  scripts/merge_bag
#  scripts/size_per_topic
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

#if(CATKIN_ENABLE_TESTING)
#  find_package(roslint REQUIRED)
#
#  roslint_custom(catkin_lint "-W2" .)
#
#  # Run roslint on Python sources
#  file(GLOB_RECURSE python_files scripts/* src/*.py)
#  roslint_python("${python_files}")
#
#  roslint_add_test()
#endif()

ament_package()