#pragma once

#include <motor_controllers/communication/communication_interface.h>

#include <string>

namespace motor_controllers {

namespace communication {

class PCA9685Communication {
 public:
  PCA9685Communication(const std::string& port, int i2cAdress);

  ~PCA9685Communication();

 public:
  void start();

  void stop();

  void setPWMFrequency(float frequency, bool externalClock = false);

  void setOscillatorFrequency(float frequency);

  void setChannelValue(uint8_t channel, uint16_t value);

 private:
  void restart();

  void sleep();

  void wake();

  void setInternalPWMFrequency(uint8_t prescale);

  void setExternalClockFrequency(uint8_t prescale);

 private:
  int file_;
  float oscillatorFrequency_;
};
}  // namespace communication
}  // namespace motor_controllers