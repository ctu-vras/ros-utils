cmake_minimum_required(VERSION 3.20.2)
project(cras_bag_tools)

find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_ros REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/extract_images
#  scripts/filter_bag
#  scripts/merge_bag
#  scripts/size_per_topic
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(cras_lint REQUIRED)
  cras_lint_common()
  cras_lint_py(EXCLUDE
    cras_bag_tools/bag_filter.py
    cras_bag_tools/message_filter.py
    cras_bag_tools/message_filters.py
  )
endif()

ament_package()