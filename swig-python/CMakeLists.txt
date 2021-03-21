project(motor_controllers_swig_wrapper)

if(POLICY CMP0078)
  cmake_policy(SET CMP0078 NEW)
endif()

if(POLICY CMP0086)
  cmake_policy(SET CMP0086 NEW)
endif()

find_package(SWIG REQUIRED  4.0 COMPONENTS python)
include(${SWIG_USE_FILE})

 
find_package(PythonLibs 3 REQUIRED)

### Common
set_property(SOURCE communication.i PROPERTY CPLUSPLUS ON)
set_property(SOURCE communication.i PROPERTY SWIG_MODULE_NAME "communication")

swig_add_library(pyMotorControllerCommunication
                 TYPE SHARED
                 LANGUAGE python
                 SOURCES communication.i)

target_link_libraries(pyMotorControllerCommunication PUBLIC MotorControllersCommunication)
target_include_directories(pyMotorControllerCommunication PUBLIC ${PYTHON_INCLUDE_PATH})
set_property(TARGET pyMotorControllerCommunication PROPERTY SWIG_USE_TARGET_INCLUDE_DIRECTORIES ON)


add_custom_command(TARGET pyMotorControllerCommunication
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/communication.py 
                                                    ${CMAKE_CURRENT_BINARY_DIR}/$<TARGET_FILE_NAME:pyMotorControllerCommunication>                                
                                                    ${CMAKE_CURRENT_LIST_DIR}/motor_controllers/)

### BCM2835
if(BUILD_BCM2835_INTERFACE)

  set_property(SOURCE bcm2835.i PROPERTY CPLUSPLUS ON)
  set_property(SOURCE bcm2835.i PROPERTY SWIG_MODULE_NAME "bcm2835")

  swig_add_library(pyMotorControllerCommunicationBCM2835
                   TYPE SHARED
                   LANGUAGE python
                   SOURCES bcm2835.i)

  target_link_libraries(pyMotorControllerCommunicationBCM2835 PUBLIC pyMotorControllerCommunication)
  target_include_directories(pyMotorControllerCommunicationBCM2835 PUBLIC ${PYTHON_INCLUDE_PATH})
  set_property(TARGET pyMotorControllerCommunicationBCM2835 PROPERTY SWIG_USE_TARGET_INCLUDE_DIRECTORIES ON)

  add_custom_command(TARGET pyMotorControllerCommunicationBCM2835
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/bcm2835.py 
                                                      ${CMAKE_CURRENT_BINARY_DIR}/$<TARGET_FILE_NAME:pyMotorControllerCommunicationBCM2835>                                
                                                      ${CMAKE_CURRENT_LIST_DIR}/motor_controllers/)

endif()


## Install the package and scripts
# https://github.com/bponsler/ros2-support/blob/master/tutorials/creating-a-mixed-cpp-and-python-package.md
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(motor_controllers PACKAGE_DIR ${CMAKE_CURRENT_LIST_DIR}/motor_controllers)

install(PROGRAMS scripts/dc_motor_with_encoder.py DESTINATION bin)