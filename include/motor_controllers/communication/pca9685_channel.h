#pragma once

#include <motor_controllers/communication/communication_channel.h>

#include <functional>
#include <string>

namespace motor_controllers {

namespace communication {

class PCA9685Channel : public CommunicationChannel {
 public:
  PCA9685Channel(uint8_t channel,
                 std::function<void(uint8_t, uint8_t*)> fctPtr);

  ~PCA9685Channel() = default;

 public:
  /**
   * @brief Set the value to send
   *
   * @param value
   */
  void setValue(float value) override;

 private:
  uint8_t channel_;
  std::function<void(uint8_t, uint8_t*)> setValue_;
};
}  // namespace communication
}  // namespace motor_controllers