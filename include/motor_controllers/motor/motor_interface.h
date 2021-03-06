#pragma once
#include <motor_controllers/communication/communication_channel.h>

#include <functional>
#include <memory>

namespace motor_controllers {
namespace motor {
class MotorInterface {
 public:
  virtual void setCommunicationChannel(
      std::unique_ptr<
          communication::CommunicationChannel,
          std::function<void(communication::CommunicationChannel*)>>) = 0;
};
}  // namespace motor
}  // namespace motor_controllers
