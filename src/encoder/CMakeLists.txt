project(motor_controllers_encoder)

add_library(motor_controller_encoder STATIC encoder.cpp)

target_include_directories(motor_controller_encoder 
                           PUBLIC 
                               $<BUILD_INTERFACE:${motor_controllers_ROOT_DIR}/include>
                                $<INSTALL_INTERFACE:include>)

target_compile_options(motor_controller_encoder PUBLIC ${SHARED_COMPILE_OPTIONS})

install(TARGETS motor_controller_encoder
        EXPORT motor_controller_encoderTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)

install(EXPORT motor_controller_encoderTargets
        FILE motor_controller_encoderTargets.cmake
        DESTINATION lib/cmake)

install(DIRECTORY ${motor_controllers_ROOT_DIR}/include/motor_controllers/encoder
        DESTINATION include/motor_controllers/)