project(motor_controllers_communication)

include(ExternalProject)

option(BUILD_PCA9685_INTERFACE "Build the PCA9685 interface" ON)
option(BUILD_BCM2835_INTERFACE "Build the BCM2835 interface" ON)

# Collect the different sources
set(motor_controllers_communication_sources communication_channel.cpp)
set(motor_controllers_communication_dependencies "")

if(BUILD_PCA9685_INTERFACE)
    find_package(i2c REQUIRED)
    list(APPEND motor_controllers_communication_sources pca9685_communication.cpp pca9685_channel.cpp)
    list(APPEND motor_controllers_communication_dependencies i2c)

endif()

if(BUILD_BCM2835_INTERFACE)
        option(BUILD_EXTERNAL_BCM2835_LIB "Download and build the BCM2835 library" ON)
        set(EXTERNAL_BCM2835_LIB_VERSION "1.68" CACHE STRING "The version of BCM2835 to use")

        if(BUILD_EXTERNAL_BCM2835_LIB)
                ExternalProject_Add(external_bcm2835_lib
                                    URL "http://www.airspayce.com/mikem/bcm2835/bcm2835-${EXTERNAL_BCM2835_LIB_VERSION}.tar.gz"
                                    INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
                                    BUILD_IN_SOURCE ON
                                    CONFIGURE_COMMAND ./configure --prefix=${CMAKE_INSTALL_PREFIX}
                                    BUILD_COMMAND make 
                                    INSTALL_COMMAND make install)
                set(BCM2835_ROOT_DIR ${CMAKE_INSTALL_PREFIX})
        endif()
        find_package(BCM2835 REQUIRED)

        list(APPEND motor_controllers_communication_sources bcm2835_communication.cpp bcm2835_channel.cpp)
        list(APPEND motor_controllers_communication_dependencies bcm2835)
        
endif()

# Declare library
add_library(motor_controllers_communication STATIC ${motor_controllers_communication_sources})
target_include_directories(motor_controllers_communication 
                           PUBLIC 
                               $<BUILD_INTERFACE:${motor_controllers_ROOT_DIR}/include>
                               $<INSTALL_INTERFACE:include>)
target_link_libraries(motor_controllers_communication PUBLIC ${motor_controllers_communication_dependencies})


install(TARGETS motor_controllers_communication
        EXPORT motor_controllers_communicationTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)

# generate and install export file
install(EXPORT motor_controllers_communicationTargets
        FILE motor_controllers_communicationTargets.cmake
        DESTINATION lib/cmake)