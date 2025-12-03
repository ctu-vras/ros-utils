# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

function(cras_lint_common)
  cmake_parse_arguments(ARG "SKIP_CMAKE_LINT;SKIP_XMLLINT;SKIP_COPYRIGHT" "MAX_LINE_LENGTH" "" ${ARGN})

  set(MAX_LINE_LENGTH 120)
  if(ARG_MAX_LINE_LENGTH)
    set(MAX_LINE_LENGTH ${ARG_MAX_LINE_LENGTH})
  endif()

  if(NOT ARG_SKIP_CMAKE_LINT)
    find_package(ament_cmake_lint_cmake REQUIRED)
    message(STATUS "Added test 'lint_cmake' to check CMake code style")
    ament_lint_cmake(MAX_LINE_LENGTH ${MAX_LINE_LENGTH})
  endif()

  if(NOT ARG_SKIP_XMLLINT)
    file(GLOB_RECURSE _source_files FOLLOW_SYMLINKS "*.xml" "*.launch")
    if(_source_files)
      find_package(ament_cmake_xmllint REQUIRED)
      message(STATUS "Added test 'xmllint' to check XML markup files")
      ament_xmllint(MAX_LINE_LENGTH ${MAX_LINE_LENGTH})
    endif()
  endif()

  if(NOT ARG_SKIP_COPYRIGHT)
    # TODO check copyright, but ament_cmake_copyright doesn't understand SPDX
  endif()
endfunction()

function(cras_lint_py)
  cmake_parse_arguments(ARG "SKIP_FLAKE8;SKIP_PEP257" "MAX_LINE_LENGTH" "EXCLUDE" ${ARGN})

  set(MAX_LINE_LENGTH 120)
  if(ARG_MAX_LINE_LENGTH)
    set(MAX_LINE_LENGTH ${ARG_MAX_LINE_LENGTH})
  endif()

  # Exclude directories that contain an AMENT_IGNORE or COLCON_IGNORE file (i.e. in-source builds)
  file(GLOB_RECURSE ament_ignores AMENT_IGNORE COLCON_IGNORE)
  foreach(file_path ${ament_ignores})
    get_filename_component(dir_path ${file_path} PATH)
    list(APPEND cras_lint_FILE_EXCLUDE ${dir_path})
  endforeach()

  set(_all_exclude "")
  if(DEFINED cras_lint_FILE_EXCLUDE)
    list(APPEND _all_exclude ${cras_lint_FILE_EXCLUDE})
  endif()

  if(ARG_EXCLUDE)
    list(APPEND _all_exclude ${ARG_EXCLUDE})
  endif()

  file(GLOB_RECURSE _source_files FOLLOW_SYMLINKS "*.py" "*.pyi")
  if(_source_files)
    if(NOT ARG_SKIP_FLAKE8)
      find_package(ament_cmake_flake8 REQUIRED)
      message(STATUS "Added test 'flake8' to check Python code syntax and style conventions")
      ament_flake8(EXCLUDE ${_all_exclude} MAX_LINE_LENGTH ${MAX_LINE_LENGTH})
    endif()

    if(NOT ARG_SKIP_PEP257)
      find_package(ament_cmake_pep257 REQUIRED)
      message(STATUS
        "Added test 'pep257' to check Python code against some of the docstring style conventions in PEP257")
      ament_pep257(--exclude ${_all_exclude})
    endif()
  endif()
endfunction()

function(cras_lint_cpp)
  cmake_parse_arguments(ARG "SKIP_CPPCHECK;SKIP_CPPLINT;SKIP_UNCRUSTIFY" "MAX_LINE_LENGTH" "EXCLUDE" ${ARGN})

  set(MAX_LINE_LENGTH 120)
  if(ARG_MAX_LINE_LENGTH)
    set(MAX_LINE_LENGTH ${ARG_MAX_LINE_LENGTH})
  endif()

  # Exclude directories that contain an AMENT_IGNORE or COLCON_IGNORE file (i.e. in-source builds)
  file(GLOB_RECURSE ament_ignores AMENT_IGNORE COLCON_IGNORE)
  foreach(file_path ${ament_ignores})
    get_filename_component(dir_path ${file_path} PATH)
    list(APPEND cras_lint_FILE_EXCLUDE ${dir_path})
  endforeach()

  file(GLOB_RECURSE _source_files FOLLOW_SYMLINKS "*.c" "*.cc" "*.cpp" "*.cxx" "*.h" "*.hh" "*.hpp" "*.hxx")
  if(_source_files)
    # Find all include directories

    set(_all_include_dirs "")
    if(DEFINED ament_cmake_cppcheck_ADDITIONAL_INCLUDE_DIRS)
      list(APPEND _all_include_dirs ${ament_cmake_cppcheck_ADDITIONAL_INCLUDE_DIRS})
    endif()

    get_directory_property(_build_targets DIRECTORY ${PROJECT_SOURCE_DIR} BUILDSYSTEM_TARGETS)
    foreach(_target ${_build_targets})
      get_target_property(_target_type ${_target} TYPE)
      if(${_target_type} STREQUAL "INTERFACE_LIBRARY")
        get_target_property(_include_dirs ${_target} INTERFACE_INCLUDE_DIRECTORIES)
      else()
        get_target_property(_include_dirs ${_target} INCLUDE_DIRECTORIES)
      endif()

      foreach(_include_dir ${_include_dirs})
        string(REGEX MATCH "^${PROJECT_SOURCE_DIR}/.*" _is_subdirectory ${_include_dir})
        string(REGEX MATCH "^\\$<.*:${PROJECT_SOURCE_DIR}/.*>$" _is_genexp_subdirectory "${_include_dir}")
        if(_is_subdirectory OR _is_genexp_subdirectory)
          list_append_unique(_all_include_dirs ${_include_dir})
        endif()
      endforeach()
    endforeach()

    # cppcheck

    if(NOT ARG_SKIP_CPPCHECK)
      find_package(ament_cmake_cppcheck REQUIRED)
      message(STATUS "Added test 'cppcheck' to perform static code analysis on C / C++ code")

      set(_all_exclude "")
      if(DEFINED ament_cmake_cppcheck_ADDITIONAL_EXCLUDE)
        list(APPEND _all_exclude ${ament_cmake_cppcheck_ADDITIONAL_EXCLUDE})
      endif()

      if(DEFINED cras_lint_FILE_EXCLUDE)
        list(APPEND _all_exclude ${cras_lint_FILE_EXCLUDE})
      endif()

      if(ARG_EXCLUDE)
        list(APPEND _all_exclude ${ARG_EXCLUDE})
      endif()

      file(GLOB_RECURSE all_exclude LIST_DIRECTORIES false ${_all_exclude})

      set(_language "C++")
      if(DEFINED ament_cmake_cppcheck_LANGUAGE)
        set(_language ${ament_cmake_cppcheck_LANGUAGE})
        message(STATUS "Configured cppcheck language: ${ament_cmake_cppcheck_LANGUAGE}")
      endif()

      message(STATUS "Configured cppcheck include dirs: ${_all_include_dirs}")
      ament_cppcheck(LANGUAGE ${_language} INCLUDE_DIRS ${_all_include_dirs} EXCLUDE ${all_exclude})
    endif()

    # cpplint

    if(NOT ARG_SKIP_CPPLINT)
      find_package(ament_cmake_cpplint REQUIRED)
      message(STATUS "Added test 'cpplint' to check C / C++ code against the Google style")

      set(_all_exclude "")
      if(DEFINED ament_cmake_cpplint_ADDITIONAL_EXCLUDE)
        list(APPEND _all_exclude ${ament_cmake_cpplint_ADDITIONAL_EXCLUDE})
      endif()

      if(DEFINED cras_lint_FILE_EXCLUDE)
        list(APPEND _all_exclude ${cras_lint_FILE_EXCLUDE})
      endif()

      if(ARG_EXCLUDE)
        list(APPEND _all_exclude ${ARG_EXCLUDE})
      endif()

      file(GLOB_RECURSE all_exclude LIST_DIRECTORIES false ${_all_exclude})

      set(_all_filters "-build/header_guard;-readability/namespace;-whitespace/braces;-runtime/references;-build/c++11")
      list(APPEND _all_filters "-legal/copyright;-whitespace/newline;-readability/nolint;-readability/todo")
      if(DEFINED ament_cmake_cpplint_FILTERS)
        set(_all_filters ${ament_cmake_cpplint_FILTERS})
      endif()
      if(DEFINED ament_cmake_cpplint_EXTRA_FILTERS)
        list(APPEND _all_filters ${ament_cmake_cpplint_EXTRA_FILTERS})
      endif()
      string(REPLACE ";" "," all_filters "${_all_filters}")

      message(STATUS "Configured cpplint exclude dirs and/or files: ${_all_exclude}")
      ament_cpplint(MAX_LINE_LENGTH ${MAX_LINE_LENGTH} --filters=${all_filters} --exclude ${all_exclude})
    endif()

    # uncrustify

    if(NOT ARG_SKIP_UNCRUSTIFY)
      find_package(ament_cmake_uncrustify REQUIRED)
      message(STATUS "Added test 'uncrustify' to check C / C++ code style")

      set(_args "")
      if(DEFINED ament_cmake_uncrustify_ADDITIONAL_ARGS)
        list(APPEND _args ${ament_cmake_uncrustify_ADDITIONAL_ARGS})
      endif()

      set(_all_exclude "")
      if(DEFINED ament_cmake_uncrustify_ADDITIONAL_EXCLUDE)
        list(APPEND _all_exclude ${ament_cmake_uncrustify_ADDITIONAL_EXCLUDE})
      endif()

      if(DEFINED cras_lint_FILE_EXCLUDE)
        list(APPEND _all_exclude ${cras_lint_FILE_EXCLUDE})
      endif()

      if(ARG_EXCLUDE)
        list(APPEND _all_exclude ${ARG_EXCLUDE})
      endif()

      file(GLOB_RECURSE all_exclude LIST_DIRECTORIES false ${_all_exclude})

      set(_config_file "${cras_lint_DIR}/../config/cras_style_uncrustify.cfg")

      message(STATUS "Configured uncrustify additional arguments: ${_args}")
      ament_uncrustify(${_args} LANGUAGE CPP EXCLUDE ${all_exclude} CONFIG_FILE ${_config_file}
        MAX_LINE_LENGTH ${MAX_LINE_LENGTH})
    endif()
  endif()
endfunction()
