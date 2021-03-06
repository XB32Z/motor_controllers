#include <bcm2835.h>
#include <motor_controllers/communication/bcm2835_pwm_channel.h>


// https://resources.pcb.cadence.com/blog/2020-pulse-width-modulation-characteristics-and-the-effects-of-frequency-and-duty-cycle 

namespace motor_controllers {

namespace communication {

BCM2835PWMChannel::BCM2835PWMChannel(const Builder& builder,
                                     std::function<void(float)> setPWMFreq)
    : IPWMSignalChannel(),
      pinNumber_(builder.pinNumber),
      pwmChannel_(builder.channel),
      range_(builder.range),
      setPWMFreq_(setPWMFreq) {}

BCM2835PWMChannel::~BCM2835PWMChannel() {
  if (!this->isCommunicationClosed()) {
    bcm2835_pwm_set_data(this->pinNumber_, 0.0);
  }
}

void BCM2835PWMChannel::setPWMFrequency(float frequency) {
  // frequency to set is the desired frequence x the range.
  // This will be divided to the oscillator freq.
  this->setPWMFreq_(frequency * this->range_);
}

void BCM2835PWMChannel::setPWM(float, float) {
  throw std::runtime_error("Cannot set PWM, please use setDutyCyle");
}

void BCM2835PWMChannel::setDutyCyle(float dutyCycle) {
  if (this->isCommunicationClosed()) {
    throw std::runtime_error(
        "BCM2835PWMChannel: communication is closed, cannot set duty cycle");
  }

  dutyCycle = std::max(std::min(dutyCycle, 1.0f), 0.0f);
  bcm2835_pwm_set_data(this->pinNumber_, dutyCycle * this->range_);
}

float BCM2835PWMChannel::getMinValue() const { return 0; }

float BCM2835PWMChannel::getMaxValue() const { return this->range_; }

void BCM2835PWMChannel::initialize() {
  bcm2835_gpio_fsel(this->pinNumber_, BCM2835_GPIO_FSEL_ALT5);
  bcm2835_pwm_set_mode(this->pwmChannel_, 1, true);
  bcm2835_pwm_set_range(this->pwmChannel_, this->range_);
}

}  // namespace communication
}  // namespace motor_controllers