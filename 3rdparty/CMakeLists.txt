cmake_minimum_required(VERSION 3.5)

project(MotorControllers3rdParty NONE)

include(ExternalProject)

if(${BUILD_EXTERNAL_BCM2835_LIB})
    # This will download and install the BCM2835 library
    ExternalProject_Add(external_bcm2835_lib
                        URL "http://www.airspayce.com/mikem/bcm2835/bcm2835-${EXTERNAL_BCM2835_LIB_VERSION}.tar.gz"
                        INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
                        BUILD_IN_SOURCE ON
                        CONFIGURE_COMMAND ${CMAKE_COMMAND} -E env CFLAGS=${EXTERNAL_BCM2835_CFLAGS} ./configure --prefix=${CMAKE_INSTALL_PREFIX} --program-suffix=mymy
                        BUILD_COMMAND CFLAGS=-fPIC make 
                        INSTALL_COMMAND make install)
endif()
