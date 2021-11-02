cmake_minimum_required(VERSION 3.0.2)
project(cras_bag_tools)

find_package(catkin REQUIRED COMPONENTS roslint)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/bag_fix_msg_defs
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  # Run roslint on Python sources
  file(GLOB python_files scripts/* src/${PROJECT_NAME}/*.py)
  roslint_python("${python_files}")
  roslint_add_test()
endif()