#include <bcm2835.h>
#include <motor_controllers/communication/bcm2835_communication.h>

#include <algorithm>
#include <cmath>
#include <map>

#define RANGE 1024

namespace motor_controllers {

namespace communication {

static const std::map<int, bcm2835PWMClockDivider> clockDividerMapping = {
    {1, BCM2835_PWM_CLOCK_DIVIDER_1},
    {2, BCM2835_PWM_CLOCK_DIVIDER_2},
    {4, BCM2835_PWM_CLOCK_DIVIDER_4},
    {8, BCM2835_PWM_CLOCK_DIVIDER_8},
    {16, BCM2835_PWM_CLOCK_DIVIDER_16},
    {32, BCM2835_PWM_CLOCK_DIVIDER_32},
    {64, BCM2835_PWM_CLOCK_DIVIDER_64},
    {128, BCM2835_PWM_CLOCK_DIVIDER_128},
    {256, BCM2835_PWM_CLOCK_DIVIDER_256},
    {512, BCM2835_PWM_CLOCK_DIVIDER_512},
    {1024, BCM2835_PWM_CLOCK_DIVIDER_1024},
    {2048, BCM2835_PWM_CLOCK_DIVIDER_2048}};

BCM2835Communication::BCM2835Communication()
    : clockDivider_(BCM2835_PWM_CLOCK_DIVIDER_16) {}

BCM2835Communication::~BCM2835Communication() { this->stop(); }

void BCM2835Communication::start() {
  if (!bcm2835_init()) {
    throw std::runtime_error("Failed to initialize BCM2835");
  }

  bcm2835_pwm_set_clock(this->clockDivider_);

  for (auto& channel : this->channels_) {
    channel.second->initialize();
  }
}

void BCM2835Communication::stop() { bcm2835_close(); }

void BCM2835Communication::setPWMFrequency(float frequency) {
  // Divides the basic 19.2MHz PWM clock.
  this->clockDivider_ =
      clockDividerMapping.at(static_cast<int>(ceil(19.2 * 10e6 / frequency)));
}

float BCM2835Communication::getMinValue() const { return 0.0; }

float BCM2835Communication::getMaxValue() const { return 1024; }

BCM2835Channel* BCM2835Communication::createChannel(uint8_t channel) {
  return new BCM2835Channel(channel, 0, this->getMaxValue());
}

}  // namespace communication
}  // namespace motor_controllers