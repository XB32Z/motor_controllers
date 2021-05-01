
project(MotorControllers3rdParty NONE)

option(BUILD_EXTERNAL_BCM2835_LIB "Download and build the BCM2835 library" ON)
option(BUILD_EXTERNAL_PIGPIO_LIB "Download and build the pigpio library" ON)

message(STATUS "Installing 3rdparty libraries")

if(${BUILD_EXTERNAL_BCM2835_LIB})
    set(EXTERNAL_BCM2835_LIB_VERSION "1.68" CACHE STRING "The version of BCM2835 to use")
    if(${BUILD_SHARED_LIBS})
        set(EXTERNAL_BCM2835_CFLAGS "-fPIC")  # Will allow to build the SHARED libraries
    endif()
    configure_file(bcm2835/CMakeLists.txt bcm2835/CMakeLists.txt)
    execute_process(COMMAND ${CMAKE_COMMAND} -DCMAKE_INSTALL_PREFIX="${CMAKE_INSTALL_PREFIX}" .
                    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bcm2835 )
    execute_process(COMMAND ${CMAKE_COMMAND} --build .
                    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bcm2835 )
endif()

if(${BUILD_EXTERNAL_PIGPIO_LIB})
    include(FetchContent)
    
    FetchContent_Declare(
        pigpio
        GIT_REPOSITORY https://github.com/joan2937/pigpio.git)

    FetchContent_MakeAvailable(pigpio)

    FetchContent_GetProperties(pigpio)
    if(NOT pigpio_POPULATED)
        FetchContent_Populate(pigpio)
        add_subdirectory(${pigpio_SOURCE_DIR} ${pigpio_BINARY_DIR})
    endif()
endif()