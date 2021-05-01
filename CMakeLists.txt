cmake_minimum_required(VERSION 3.5)
project(motor_controllers)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(SHARED_COMPILE_OPTIONS -Wall -Wextra -Wpedantic -O3)
endif()


# Dependencies
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(ament_cmake REQUIRED)

option(BUILD_SHARED_LIBS "Builds the library as SHARED. This is required to have the python wrapper." OFF)

option(BUILD_3RDPARTY "Download and install the 3rdparty libraries. Will do so at configure time." ON)
if(BUILD_3RDPARTY)
  add_subdirectory(3rdparty)
endif()


# Add source subdirectory
set(motor_controllers_ROOT_DIR ${CMAKE_CURRENT_LIST_DIR})
add_subdirectory(src)


# SWIG wrapping
option(BUILD_PYTHON_WRAPPER "Build a library wrapper for python" OFF)
if (BUILD_PYTHON_WRAPPER)
  if (NOT BUILD_SHARED_LIBS)
    message(FATAL_ERROR "Cannot build the Python wrapper with STATIC libraries. Set BUILD_SHARED_LIBS to ON.")
  endif()
  add_subdirectory(swig-python)
endif()


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
