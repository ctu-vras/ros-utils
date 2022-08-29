cmake_minimum_required(VERSION 2.8.3)
project(cras_py_common)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)

  # catkin_lint - checks validity of package.xml and CMakeLists.txt
  if (roslaunch_VERSION VERSION_GREATER_EQUAL 1.15.0)
    set(distro noetic)
  else()
    set(distro melodic)
  endif()

  # ROS buildfarm calls this without any environment, so we have problems reading the list of packages from env
  # roslint_custom(catkin_lint "-W2" --rosdistro "${distro}" .)

  # add_dependencies(run_tests roslint_${PROJECT_NAME})
endif()