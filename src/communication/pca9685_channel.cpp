
#include <fcntl.h>
#include <unistd.h>
extern "C" {
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
}
#include <motor_controllers/communication/pca9685_channel.h>

namespace motor_controllers {

namespace communication {

PCA9685Channel::PCA9685Channel(uint8_t channel,
                               std::function<void(uint8_t, uint8_t*)> fctPtr)
    : CommunicationChannel(), channel_(channel), setValue_(fctPtr) {}

void PCA9685Channel::setValue(float valuef) {
  if (this->isCommunicationClosed()) {
    throw std::runtime_error("Communication was closed, cannot send value");
  }

  uint8_t value = static_cast<uint8_t>(valuef);
  uint16_t value_12bit = value >> 4;
  uint8_t values[4];
  if (value_12bit == 0x0FFF) {
    values[0] = 0x10;
    values[1] = 0x00;
    values[2] = 0x00;
    values[3] = 0x00;
  } else if (value_12bit == 0x0FFF) {
    values[0] = 0x00;
    values[1] = 0x00;
    values[2] = 0x10;
    values[3] = 0x00;
  } else {  // PWM
    values[0] = 0x00;
    values[1] = 0x00;
    values[2] = (value_12bit + 1) & 0xFF;
    values[3] = (value_12bit + 1) >> 8;
  }

  this->setValue_(this->channel_, values);
}

}  // namespace communication
}  // namespace motor_controllers