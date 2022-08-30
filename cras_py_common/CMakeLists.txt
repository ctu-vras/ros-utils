cmake_minimum_required(VERSION 3.7.2)
project(cras_py_common)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)

  # catkin_lint - checks validity of package.xml and CMakeLists.txt
  # ROS buildfarm calls this without any environment, so we have problems reading the list of packages from env
  # see https://github.com/ros-infrastructure/ros_buildfarm/issues/923
  if(DEFINED ENV{ROS_ROOT})
    roslint_custom(catkin_lint "-W2" .)
  endif()
endif()