#pragma once

namespace motor_controllers {
namespace communication {

class CommunicationInterface {
  virtual void start() = 0;

  virtual void stop() = 0;
};
}  // namespace communication

}  // namespace motor_controllers