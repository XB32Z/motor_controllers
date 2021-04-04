%module py_bcm2835

%feature("flatnested", "1"); // https://stackoverflow.com/questions/42892858/warning-325-nested-class-not-currently-supported-proxy-ignored

%include "communication.i"

%{
  #include <motor_controllers/communication/bcm2835_interface.h>
%}


wrap_unique_ptr(BCM2835PWMChannelPtr, motor_controllers::communication::BCM2835PWMChannel, std::function<void(motor_controllers::communication::ISignalChannel*)>);
wrap_unique_ptr(BCM2835BinaryChannelPtr, motor_controllers::communication::BCM2835BinaryChannel, std::function<void(motor_controllers::communication::ISignalChannel*)>);


namespace motor_controllers {

namespace communication {
    %rename (BCM2835PWMChannelConfiguration) BCM2835PWMChannel::Configuration;
    %rename (BCM2835BinaryChannelConfiguration) BCM2835BinaryChannel::Configuration;

    %template(BCM2835PWMChannelBuilder) ChannelBuilder< motor_controllers::communication::BCM2835PWMChannel, motor_controllers::communication::BCM2835PWMChannel::Configuration >;
    %template(BCM2835BinaryChannelBuilder) ChannelBuilder<motor_controllers::communication::BCM2835BinaryChannel, motor_controllers::communication::BCM2835BinaryChannel::Configuration>;
}
}
%include <motor_controllers/communication/bcm2835_binary_channel.h>
%include <motor_controllers/communication/bcm2835_pwm_channel.h>
%include <motor_controllers/communication/bcm2835_interface.h>