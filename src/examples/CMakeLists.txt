project(MotorControllersExamples)


if(BUILD_PCA9685_INTERFACE)
    add_subdirectory(pca9685)
endif()

if(BUILD_BCM2835_INTERFACE)
    add_subdirectory(bcm2835)
endif()

if(BUILD_PIGPIO_INTERFACE)
    add_subdirectory(pigpio)
endif()