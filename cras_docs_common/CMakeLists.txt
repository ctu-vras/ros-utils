cmake_minimum_required(VERSION 3.10.2)
project(cras_docs_common)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

install(DIRECTORY src/${PROJECT_NAME}/sphinx-themes DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)

  roslint_custom(catkin_lint "-W2" .)
  roslint_add_test()  # Move this outside the if() block when more roslint tests are added
endif()
