#include <bcm2835.h>
#include <motor_controllers/communication/bcm2835_channel.h>

namespace motor_controllers {

namespace communication {

BCM2835Channel::BCM2835Channel(uint8_t pinNumber, uint8_t channel,
                               uint8_t range)
    : CommunicationChannel(),
      pinNumber_(pinNumber),
      pwmChannel_(channel),
      range_(range) {}

void BCM2835Channel::setValue(float value) {
  bcm2835_pwm_set_data(this->pinNumber_, value);
  bcm2835_delay(1);
}

void BCM2835Channel::initialize() {
  bcm2835_gpio_fsel(this->pinNumber_, BCM2835_GPIO_FSEL_ALT5);
  bcm2835_pwm_set_mode(this->pwmChannel_, 1, true);
  bcm2835_pwm_set_range(this->pwmChannel_, this->range_);
}

}  // namespace communication
}  // namespace motor_controllers