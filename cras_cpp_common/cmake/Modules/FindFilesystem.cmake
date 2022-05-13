# Distributed under the OSI-approved BSD 3-Clause License.
# Base on https://github.com/vector-of-bool/CMakeCM/blob/master/modules/FindFilesystem.cmake

#[=======================================================================[.rst:

FindFilesystem
##############

This module supports the C++17 standard library's filesystem utilities. Use the
:imp-target:`std::filesystem` imported target to enable support of it.

Options
*******

The ``COMPONENTS`` argument to this module supports the following values:

.. find-component:: Boost
    :name: fs.Boost

    Allows the module to find the "boost" Filesystem TS version of the
    Filesystem library. This is the library that should be used with the
    ``boost::filesystem`` namespace.

.. find-component:: Experimental
    :name: fs.Experimental

    Allows the module to find the "experimental" Filesystem TS version of the
    Filesystem library. This is the library that should be used with the
    ``std::experimental::filesystem`` namespace.

.. find-component:: Final
    :name: fs.Final

    Finds the final C++17 standard version of the filesystem library.

If no components are provided, behaves as if the
:find-component:`fs.Final` component was specified.

If more components are defined, it first looks for ``Final``, then falls back to
``Experimental``, and then to ``Boost``. If a component is found, all remaining
components are skipped.


Imported Targets
****************

.. imp-target:: std::filesystem

    The ``std::filesystem`` imported target is defined when any requested
    version of the C++ filesystem library has been found, whether it is
    *Experimental* or *Final*.

    If no version of the filesystem library is available, this target will not
    be defined.

    The target defines all variables from the following section as preprocessor
    directives so that they can be used in the sources.

    .. note::
        For example, you can do the following in your .cpp file:
        #include CXX_FILESYSTEM_INCLUDE
        namespace fs = CXX_FILESYSTEM_NAMESPACE;

    .. note::
        This target can have ``cxx_std_17`` as an ``INTERFACE``
        :ref:`compile language standard feature <req-lang-standards>`. Linking
        to this target will automatically enable C++17 if no later standard
        version is already required on the linking target and the found
        component is not ``Boost``.


.. _fs.variables:

Variables
*********

.. variable:: CXX_FILESYSTEM_TYPE

    Set to ``Final`` when the :find-component:`fs.Final` version of C++
    filesystem library was found and so on for ``Experimental`` and ``Boost``.

.. variable:: CXX_FILESYSTEM_HAVE_FS

    Set to ``TRUE`` when a filesystem header was found.

.. variable:: CXX_FILESYSTEM_HEADER

    Set to either ``filesystem``, ``experimental/filesystem`` or
    ``boost/filesystem.hpp`` depending on which component was found.

.. variable:: CXX_FILESYSTEM_INCLUDE

    Set to either ``<filesystem>``, ``<experimental/filesystem>`` or
    ``<boost/filesystem.hpp>`` depending on which component was found.

.. variable:: CXX_FILESYSTEM_NAMESPACE

    Set to either ``std::filesystem``, ``std::experimental::filesystem`` or
    ``boost::filesystem`` depending on which component was found.


Examples
********

Using `find_package(Filesystem)` with no component arguments:

.. code-block:: cmake

    find_package(Filesystem REQUIRED)

    add_executable(my-program main.cpp)
    target_link_libraries(my-program PRIVATE std::filesystem)


#]=======================================================================]


if(TARGET std::filesystem)
  # This module has already been processed. Don't do it again.
  return()
endif()

# All of our tests require C++17 or later
set(CMAKE_CXX_STANDARD 17)

cmake_policy(PUSH)
# support if(IN_LIST) operator
cmake_policy(SET CMP0057 NEW)
# pass CMAKE_CXX_STANDARD to try_compile and check_include_file_cxx
cmake_policy(SET CMP0067 NEW)

include(CMakePushCheckState)
include(CheckIncludeFileCXX)
include(CheckCXXSourceCompiles)

cmake_push_check_state()

set(CMAKE_REQUIRED_QUIET ${Filesystem_FIND_QUIETLY})

# Normalize and check the component list we were given
set(want_components ${Filesystem_FIND_COMPONENTS})
if(Filesystem_FIND_COMPONENTS STREQUAL "")
  set(want_components Final)
endif()

# Warn on any unrecognized components
set(extra_components ${want_components})
list(REMOVE_ITEM extra_components Final Experimental Boost)
foreach(component IN LISTS extra_components)
  message(WARNING "Extraneous find_package component for Filesystem: ${component}")
endforeach()

# Detect which of Experimental and Final we should look for
set(find_boost TRUE)
set(find_experimental TRUE)
set(find_final TRUE)
if(NOT "Final" IN_LIST want_components)
  set(find_final FALSE)
endif()
if(NOT "Experimental" IN_LIST want_components)
  set(find_experimental FALSE)
endif()
if(NOT "Boost" IN_LIST want_components)
  set(find_boost FALSE)
endif()

if(find_final)
  check_include_file_cxx("filesystem" _CXX_FILESYSTEM_HAVE_HEADER)
  mark_as_advanced(_CXX_FILESYSTEM_HAVE_HEADER)
  if(_CXX_FILESYSTEM_HAVE_HEADER)
    # We found the non-experimental header. Don't bother looking for the
    # other ones.
    set(find_experimental FALSE)
    set(find_boost FALSE)
  endif()
else()
  set(_CXX_FILESYSTEM_HAVE_HEADER FALSE)
endif()

if(find_experimental)
  check_include_file_cxx("experimental/filesystem" _CXX_FILESYSTEM_HAVE_EXPERIMENTAL_HEADER)
  mark_as_advanced(_CXX_FILESYSTEM_HAVE_EXPERIMENTAL_HEADER)
  if(_CXX_FILESYSTEM_HAVE_EXPERIMENTAL_HEADER)
    # We found the non-experimental header. Don't bother looking for the
    # other ones.
    set(find_boost FALSE)
  endif()
else()
  set(_CXX_FILESYSTEM_HAVE_EXPERIMENTAL_HEADER FALSE)
endif()

if(find_boost)
  find_package(Boost COMPONENTS filesystem)
  set(_BOOST_FILESYSTEM_HAVE_HEADER "${Boost_FILESYSTEM_FOUND}")
else()
  set(_BOOST_FILESYSTEM_HAVE_HEADER FALSE)
endif()

if(${_CXX_FILESYSTEM_HAVE_HEADER})
  set(_have_fs TRUE)
  set(_fs_header filesystem)
  set(_fs_namespace std::filesystem)
  set(_fs_type Final)
elseif(${_CXX_FILESYSTEM_HAVE_EXPERIMENTAL_HEADER})
  set(_have_fs TRUE)
  set(_fs_header experimental/filesystem)
  set(_fs_namespace std::experimental::filesystem)
  set(_fs_type Experimental)
elseif(${_BOOST_FILESYSTEM_HAVE_HEADER})
  set(_have_fs TRUE)
  set(_fs_header boost/filesystem.hpp)
  set(_fs_namespace boost::filesystem)
  set(_fs_type Boost)
else()
  set(_have_fs FALSE)
endif()

set(CXX_FILESYSTEM_HAVE_FS ${_have_fs} CACHE BOOL "TRUE if we have the C++ filesystem headers")
set(CXX_FILESYSTEM_HEADER ${_fs_header} CACHE STRING "The header that should be included to obtain the filesystem APIs")
set(CXX_FILESYSTEM_INCLUDE "<${_fs_header}>" CACHE STRING "The #include argument to use in C++ files")
set(CXX_FILESYSTEM_NAMESPACE ${_fs_namespace} CACHE STRING "The C++ namespace that contains the filesystem APIs")
set(CXX_FILESYSTEM_TYPE ${_fs_type} CACHE STRING "The component that satisfied the filesystem API")

set(_found FALSE)

if(CXX_FILESYSTEM_HAVE_FS)
  # We have some filesystem library available. Do link checks
  string(CONFIGURE [[
        #include @CXX_FILESYSTEM_INCLUDE@

        int main() {
            auto cwd = @CXX_FILESYSTEM_NAMESPACE@::current_path();
            return static_cast<int>(cwd.string().size());
        }
    ]] code @ONLY)

  # Try to compile a simple filesystem program without any linker flags
  check_cxx_source_compiles("${code}" CXX_FILESYSTEM_NO_LINK_NEEDED)

  set(can_link ${CXX_FILESYSTEM_NO_LINK_NEEDED})

  if(NOT can_link)
    cmake_push_check_state()
    # Add the libstdc++ flag
    set(CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES} -lstdc++fs)
    check_cxx_source_compiles("${code}" CXX_FILESYSTEM_STDCPPFS_NEEDED)
    set(can_link ${CXX_FILESYSTEM_STDCPPFS_NEEDED})
    cmake_pop_check_state()
    if(NOT can_link)
      cmake_push_check_state()
      # Try the libc++ flag      
      set(CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES} -lc++fs)
      check_cxx_source_compiles("${code}" CXX_FILESYSTEM_CPPFS_NEEDED)
      set(can_link ${CXX_FILESYSTEM_CPPFS_NEEDED})
      cmake_pop_check_state()
      if(NOT can_link AND find_boost)
        # Try Boost
	      cmake_push_check_state()
	      set(CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES} Boost::filesystem)
	      set(CMAKE_REQUIRED_INCLUDES ${CMAKE_REQUIRED_INCLUDES} ${Boost_INCLUDE_DIRS})
        check_cxx_source_compiles("${code}" CXX_FILESYSTEM_BOOST_NEEDED)
        set(can_link ${CXX_FILESYSTEM_BOOST_NEEDED})
	      cmake_pop_check_state()
      endif()
    endif()
  endif()

  if(can_link)
    add_library(std::filesystem INTERFACE IMPORTED)
    set(_found TRUE)

    if(CXX_FILESYSTEM_NO_LINK_NEEDED)
      set_property(TARGET std::filesystem APPEND PROPERTY INTERFACE_COMPILE_DEFINITIONS cxx_std_17)
    elseif(CXX_FILESYSTEM_STDCPPFS_NEEDED)
      set_property(TARGET std::filesystem APPEND PROPERTY INTERFACE_LINK_LIBRARIES -lstdc++fs)
      set_property(TARGET std::filesystem APPEND PROPERTY INTERFACE_COMPILE_DEFINITIONS cxx_std_17)
    elseif(CXX_FILESYSTEM_CPPFS_NEEDED)
      set_property(TARGET std::filesystem APPEND PROPERTY INTERFACE_LINK_LIBRARIES -lc++fs)
      set_property(TARGET std::filesystem APPEND PROPERTY INTERFACE_COMPILE_DEFINITIONS cxx_std_17)
    elseif(CXX_FILESYSTEM_BOOST_NEEDED)
      set_property(TARGET std::filesystem APPEND PROPERTY INTERFACE_LINK_LIBRARIES Boost::filesystem)
      set_property(TARGET std::filesystem APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIRS})
    endif()
  endif()
endif()

cmake_pop_check_state()

set(Filesystem_FOUND ${_found} CACHE BOOL "TRUE if we can compile and link a program using std::filesystem" FORCE)

if(Filesystem_FOUND)
  set_property(TARGET std::filesystem APPEND PROPERTY INTERFACE_COMPILE_DEFINITIONS CXX_FILESYSTEM_FOUND=${Filesystem_FOUND})
  set_property(TARGET std::filesystem APPEND PROPERTY INTERFACE_COMPILE_DEFINITIONS CXX_FILESYSTEM_HEADER=${CXX_FILESYSTEM_HEADER})
  set_property(TARGET std::filesystem APPEND PROPERTY INTERFACE_COMPILE_DEFINITIONS CXX_FILESYSTEM_INCLUDE=${CXX_FILESYSTEM_INCLUDE})
  set_property(TARGET std::filesystem APPEND PROPERTY INTERFACE_COMPILE_DEFINITIONS CXX_FILESYSTEM_NAMESPACE=${CXX_FILESYSTEM_NAMESPACE})
  set_property(TARGET std::filesystem APPEND PROPERTY INTERFACE_COMPILE_DEFINITIONS CXX_FILESYSTEM_TYPE=${CXX_FILESYSTEM_TYPE})
endif()

if(Filesystem_FIND_REQUIRED AND NOT Filesystem_FOUND)
  message(FATAL_ERROR "Cannot Compile simple program using std::filesystem")
endif()

cmake_policy(POP)
