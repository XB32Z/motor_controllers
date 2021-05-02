project(MotorControllersCommunication)

option(BUILD_PCA9685_INTERFACE "Build the PCA9685 interface" ON)
option(BUILD_BCM2835_INTERFACE "Build the BCM2835 interface" ON)
option(BUILD_PIGPIO_INTERFACE "Build the pigpio interface" ON)

# Collect the different sources
set(${PROJECT_NAME}_sources i_signal_channel.cpp)
set(${PROJECT_NAME}_dependencies "")

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

if(BUILD_PCA9685_INTERFACE)
    find_package(i2c REQUIRED)
    list(APPEND ${PROJECT_NAME}_sources pca9685/pca9685_interface.cpp 
                                        pca9685/pca9685_channel.cpp)
    list(APPEND ${PROJECT_NAME}_dependencies i2c)

endif()

if(BUILD_BCM2835_INTERFACE)
        find_package(BCM2835 REQUIRED)

        list(APPEND ${PROJECT_NAME}_sources bcm2835/bcm2835_interface.cpp 
                                            bcm2835/bcm2835_pwm_channel.cpp 
                                            bcm2835/bcm2835_binary_channel.cpp)
        list(APPEND ${PROJECT_NAME}_dependencies bcm2835)
        
endif()

if(BUILD_PIGPIO_INTERFACE)
        if(TARGET pigpio)
                # the pigpio library does not define INTERFACE_INCLUDE_DIRECTORIES on the build tree.
                include_directories(${pigpio_SOURCE_DIR})
        else()
                find_package(pigpio REQUIRED)
        endif()
        
        list(APPEND ${PROJECT_NAME}_sources pigpio/pigpio_interface.cpp 
                                            pigpio/pigpio_pwm_channel.cpp 
                                            pigpio/pigpio_binary_channel.cpp)
        list(APPEND ${PROJECT_NAME}_dependencies pigpio)
        
endif()

# Declare library
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_sources})
target_include_directories(${PROJECT_NAME} 
                           PUBLIC 
                               $<BUILD_INTERFACE:${motor_controllers_ROOT_DIR}/include>
                               $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} 
                      PUBLIC ${${PROJECT_NAME}_dependencies}
                      PRIVATE Threads::Threads)
target_compile_options(${PROJECT_NAME} PUBLIC ${SHARED_COMPILE_OPTIONS})

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)

install(EXPORT ${PROJECT_NAME}Targets
        FILE ${PROJECT_NAME}Targets.cmake
        DESTINATION lib/cmake)
