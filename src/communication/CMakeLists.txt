project(motor_controllers_communication)

option(BUILD_PCA9685_INTERFACE "Build the PCA9685 interface" ON)
option(BUILD_BCM2835_INTERFACE "Build the BCM2835 interface" ON)

# Collect the different sources
set(motor_controllers_communication_sources i_signal_channel.cpp)
set(motor_controllers_communication_dependencies "")

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

if(BUILD_PCA9685_INTERFACE)
    find_package(i2c REQUIRED)
    list(APPEND motor_controllers_communication_sources pca9685_interface.cpp pca9685_channel.cpp)
    list(APPEND motor_controllers_communication_dependencies i2c)

endif()

if(BUILD_BCM2835_INTERFACE)
        find_package(BCM2835 REQUIRED)

        list(APPEND motor_controllers_communication_sources bcm2835_interface.cpp bcm2835_pwm_channel.cpp bcm2835_binary_channel.cpp)
        list(APPEND motor_controllers_communication_dependencies bcm2835)
        
endif()

# Declare library
add_library(motor_controllers_communication STATIC ${motor_controllers_communication_sources})
target_include_directories(motor_controllers_communication 
                           PUBLIC 
                               $<BUILD_INTERFACE:${motor_controllers_ROOT_DIR}/include>
                                $<INSTALL_INTERFACE:include>)
target_link_libraries(motor_controllers_communication 
                      PUBLIC ${motor_controllers_communication_dependencies}
                      PRIVATE Threads::Threads)
target_compile_options(motor_controllers_communication PUBLIC ${SHARED_COMPILE_OPTIONS})

install(TARGETS motor_controllers_communication
        EXPORT motor_controllers_communicationTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)

install(EXPORT motor_controllers_communicationTargets
        FILE motor_controllers_communicationTargets.cmake
        DESTINATION lib/cmake)

install(DIRECTORY ${motor_controllers_ROOT_DIR}/include/motor_controllers/communication
        DESTINATION include/motor_controllers/)