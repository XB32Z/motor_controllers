%module py_motor_controller_communication

%include "stdint.i"
%include "std_unique_ptr.i"
%include "std_future.i"

%{
  #include <motor_controllers/communication/i_communication_interface.h>
  #include <motor_controllers/communication/i_pwm_signal_channel.h>
  #include <motor_controllers/communication/i_binary_signal_channel.h>
%}

wrap_future(FutureBinarySignal, motor_controllers::communication::BinarySignal)

wrap_unique_ptr(PWMChannelPtr, motor_controllers::communication::IPWMSignalChannel, std::function<void(motor_controllers::communication::IPWMSignalChannel*)>);
wrap_unique_ptr(BinaryChannelPtr, motor_controllers::communication::IBinarySignalChannel, std::function<void(motor_controllers::communication::IBinarySignalChannel*)>);

%include <motor_controllers/communication/i_communication_interface.h>
%include <motor_controllers/communication/i_signal_channel.h>
%include <motor_controllers/communication/i_pwm_signal_channel.h>
%include <motor_controllers/communication/i_binary_signal_channel.h>
%include <motor_controllers/communication/channel_builder.h>