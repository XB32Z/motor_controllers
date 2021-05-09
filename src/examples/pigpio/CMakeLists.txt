project(MotorControllersPiGPIOExamples)


add_executable(pigpio_dc_motor pigpio_dc_motor.cpp)
target_link_libraries(pigpio_dc_motor 
                      PUBLIC MotorControllersCommunication MotorControllersEncoder)


add_executable(pigpio_dc_motor_factory pigpio_dc_motor_factory.cpp)
target_link_libraries(pigpio_dc_motor_factory 
                      PUBLIC MotorControllersCommunication MotorControllersEncoder MotorControllersMotor)
                      
install(TARGETS pigpio_dc_motor pigpio_dc_motor_factory DESTINATION bin)