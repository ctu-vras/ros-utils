# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.20.2)
project(cras_lint NONE)

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_test REQUIRED)

if(BUILD_TESTING)
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cras_lint.cmake)
  cras_lint_common()
endif()

ament_package(
  CONFIG_EXTRAS "cras_lint-extras.cmake"
)

install(
  DIRECTORY cmake config
  DESTINATION share/${PROJECT_NAME}
)
