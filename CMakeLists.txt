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
  set(SHARED_COMPILE_OPTIONS -Wall -Wextra -Wpedantic)
endif()


# Dependencies
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(ament_cmake REQUIRED)

option(BUILD_3RDPARTY "Download and install the 3rdparty libraries. Will do so at configure time" ON)
if(BUILD_3RDPARTY)

  option(BUILD_EXTERNAL_BCM2835_LIB "Download and build the BCM2835 library" ON)
  set(EXTERNAL_BCM2835_LIB_VERSION "1.68" CACHE STRING "The version of BCM2835 to use")

  message(STATUS "Installing 3rdparty libraries")
  configure_file(3rdparty/CMakeLists.txt 3rdparty/CMakeLists.txt)
  execute_process(COMMAND ${CMAKE_COMMAND} -DCMAKE_INSTALL_PREFIX="${CMAKE_INSTALL_PREFIX}" .
                  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/3rdparty )
  execute_process(COMMAND ${CMAKE_COMMAND} --build .
                  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/3rdparty )
endif()


# Add source subdirectory
set(motor_controllers_ROOT_DIR ${CMAKE_CURRENT_LIST_DIR})

add_subdirectory(src)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
