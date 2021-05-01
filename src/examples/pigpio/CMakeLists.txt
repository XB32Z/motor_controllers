project(MotorControllersPiGPIOExamples)


add_executable(pigpio_dc_motor pigpio_dc_motor.cpp)
target_link_libraries(pigpio_dc_motor 
                      PUBLIC MotorControllersCommunication MotorControllersEncoder)
                      
install(TARGETS pigpio_dc_motor DESTINATION bin)