project(motor_controllers_communication)

option(BUILD_PCA9685 "Build the PCA9685 interface" ON)

if(BUILD_PCA9685)
    find_package(i2c REQUIRED)
    add_library(pca9685_interface pca9685_communication.cpp)
    target_include_directories( pca9685_interface 
                                PUBLIC 
                                    $<BUILD_INTERFACE:${motor_controllers_ROOT_DIR}/include>
                                    $<INSTALL_INTERFACE:include>)
    target_link_libraries(pca9685_interface i2c)


    install(TARGETS pca9685_interface
            EXPORT pca9685_interfaceTargets
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib
            RUNTIME DESTINATION bin)

    # generate and install export file
    install(EXPORT pca9685_interfaceTargets
            FILE pca9685_interfaceTargets.cmake
            DESTINATION lib/cmake)

endif()