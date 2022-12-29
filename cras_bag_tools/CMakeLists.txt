cmake_minimum_required(VERSION 3.7.2)
project(cras_bag_tools)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/fix_msg_defs
  scripts/merge_bag
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)

  # catkin_lint - checks validity of package.xml and CMakeLists.txt
  # ROS buildfarm calls this without any environment, so we have problems reading the list of packages from env
  # see https://github.com/ros-infrastructure/ros_buildfarm/issues/923
  if(DEFINED ENV{ROS_ROOT})
    roslint_custom(catkin_lint "-W2" .)
  endif()

  # Run roslint on Python sources
  file(GLOB python_files scripts/* src/${PROJECT_NAME}/*.py)
  roslint_python("${python_files}")
  roslint_add_test()
endif()