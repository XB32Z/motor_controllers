project(motor_controllers_examples)


add_executable(dc_motor_with_encoder dc_motor_with_encoder.cpp)
target_link_libraries(dc_motor_with_encoder 
                      PUBLIC motor_controllers_communication motor_controller_encoder)


add_executable(simple_dc_motor simple_dc_motor.cpp)
target_link_libraries(simple_dc_motor 
                      PUBLIC motor_controllers_communication)

add_executable(simple_encoder simple_encoder.cpp)
target_link_libraries(simple_encoder 
                      PUBLIC motor_controllers_communication motor_controller_encoder)


add_executable(simple_event_detect_pins simple_event_detect_pins.cpp)
target_link_libraries(simple_event_detect_pins 
                      PUBLIC motor_controllers_communication)


add_executable(simple_read_pins simple_read_pins.cpp)
target_link_libraries(simple_read_pins 
                      PUBLIC motor_controllers_communication)


add_executable(simple_servo simple_servo.cpp)
target_link_libraries(simple_servo 
                      PUBLIC motor_controllers_communication)