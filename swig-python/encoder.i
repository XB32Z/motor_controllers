%module py_bcm2835

%include "communication.i"

%{
  #include <motor_controllers/encoder/encoder.h>
%}

namespace motor_controllers {
  namespace encoder {
    // Ignore the constructors because SWIG will not call move or cast the unique_ptrs
    %ignore Encoder::Encoder(
          communication::IBinarySignalChannel::Ref channelA, 
          communication::IBinarySignalChannel::Ref channelB,
          unsigned int resolution);

    %ignore Encoder::Encoder(communication::IBinarySignalChannel::Ref,unsigned int);

  }
}

%include <motor_controllers/encoder/encoder.h>

namespace motor_controllers {
  namespace encoder {
    // Redefine the constructors with a reference and move the unique ptr
    %extend Encoder{
      Encoder(
          communication::IBinarySignalChannel::Ref& channelA, 
          communication::IBinarySignalChannel::Ref& channelB,
          unsigned int resolution) {
            auto encoder = new motor_controllers::encoder::Encoder(
              std::move(channelA), std::move(channelB), resolution);
            return encoder;
      }
    };

    %extend Encoder{
      Encoder(
          communication::IBinarySignalChannel::Ref& channel, 
          unsigned int resolution) {
            auto encoder = new motor_controllers::encoder::Encoder(
              std::move(channel), resolution);
            return encoder;
      }
    };

  }
}