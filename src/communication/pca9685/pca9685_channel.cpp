
#include <fcntl.h>
#include <unistd.h>
extern "C" {
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
}
#include <motor_controllers/communication/pca9685/pca9685_channel.h>

namespace motor_controllers {

namespace communication {

PCA9685Channel::PCA9685Channel(const Configuration& buidler,
                               std::function<void(uint8_t, uint8_t*)> setValue,
                               std::function<void(float)> setPWMFreq)
    : IPWMSignalChannel(),
      channel_(buidler.channelId),
      range_(buidler.range),
      setValue_(setValue),
      setPWMFreq_(setPWMFreq) {}

void PCA9685Channel::setPWMFrequency(float frequency) {
  this->setPWMFreq_(frequency);
}

void PCA9685Channel::setPWM(float start, float end) {
  if (this->isCommunicationClosed()) {
    throw std::runtime_error("Communication was closed, cannot send value");
  }

  uint16_t startVal = static_cast<uint16_t>(start);
  uint16_t endVal = static_cast<uint16_t>(end);

  uint8_t values[4];
  values[0] = startVal;
  values[1] = startVal >> 8;
  values[2] = endVal;
  values[3] = endVal >> 8;

  this->setValue_(this->channel_, values);
}

void PCA9685Channel::setDutyCycle(float dutyCycle) {
  dutyCycle = std::max(std::min(dutyCycle, 1.0f), 0.0f);
  this->setPWM(0, dutyCycle * this->range_);
}

float PCA9685Channel::getMinValue() const { return static_cast<float>(0x0000); }

float PCA9685Channel::getMaxValue() const { return static_cast<float>(0x0FFF); }

}  // namespace communication
}  // namespace motor_controllers
