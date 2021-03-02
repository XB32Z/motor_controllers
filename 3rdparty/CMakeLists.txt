cmake_minimum_required(VERSION 3.5)

project(motor_controllers_3rdparty NONE)

include(ExternalProject)

if(${BUILD_EXTERNAL_BCM2835_LIB})
    # This will dowload and install the BCM2835 library
    ExternalProject_Add(external_bcm2835_lib
                        URL "http://www.airspayce.com/mikem/bcm2835/bcm2835-${EXTERNAL_BCM2835_LIB_VERSION}.tar.gz"
                        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
                        BUILD_IN_SOURCE ON
                        CONFIGURE_COMMAND ./configure --prefix=${CMAKE_INSTALL_PREFIX}
                        BUILD_COMMAND make 
                        INSTALL_COMMAND make install)
endif()
