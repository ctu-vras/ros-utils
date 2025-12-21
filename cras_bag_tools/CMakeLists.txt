cmake_minimum_required(VERSION 3.10.2)
project(cras_bag_tools)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/extract_images
  scripts/extract_pcd
  scripts/filter_bag
  scripts/fix_msg_defs
  scripts/merge_bag
  scripts/size_per_topic
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)

  roslint_custom(catkin_lint "-W2" .)

  # Run roslint on Python sources
  file(GLOB_RECURSE python_files scripts/* src/*.py)
  roslint_python("${python_files}")

  roslint_add_test()
endif()
