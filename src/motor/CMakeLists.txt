project(MotorControllersMotor)


add_library(${PROJECT_NAME} dc_motor.cpp)
target_include_directories(${PROJECT_NAME} 
                           PUBLIC 
                               $<BUILD_INTERFACE:${motor_controllers_ROOT_DIR}/include>
                               $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} 
                      PUBLIC MotorControllersCommunication MotorControllersEncoder
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
