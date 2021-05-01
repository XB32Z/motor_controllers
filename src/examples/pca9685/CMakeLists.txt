project(MotorControllersPCA9685Examples)


add_executable(simple_servo simple_servo.cpp)
target_link_libraries(simple_servo 
                      PUBLIC MotorControllersCommunication)
