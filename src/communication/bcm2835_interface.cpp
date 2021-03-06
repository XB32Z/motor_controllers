#include <bcm2835.h>
#include <motor_controllers/communication/bcm2835_interface.h>

#include <algorithm>
#include <cmath>
#include <map>

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

BCM2835Interface::BCM2835Interface()
    : clockDivider_(BCM2835_PWM_CLOCK_DIVIDER_16) {}

BCM2835Interface::~BCM2835Interface() { this->stop(); }

void BCM2835Interface::start() {
  if (!bcm2835_init()) {
    throw std::runtime_error("Failed to initialize BCM2835");
  }

  bcm2835_pwm_set_clock(this->clockDivider_);

  for (auto& channel :
       this->ChannelBuilder<BCM2835PWMChannel,
                            BCM2835PWMChannel::Builder>::channels_) {
    channel->initialize();
  }

  for (auto& channel :
       this->ChannelBuilder<BCM2835BinaryChannel,
                            BCM2835BinaryChannel::Builder>::channels_) {
    channel->initialize();
  }
}

void BCM2835Interface::stop() { 

  for (auto& channel :
       this->ChannelBuilder<BCM2835PWMChannel,
                            BCM2835PWMChannel::Builder>::channels_) {
    channel->setDutyCyle(0.0);
  }

  for (auto& channel :
       this->ChannelBuilder<BCM2835BinaryChannel,
                            BCM2835BinaryChannel::Builder>::channels_) {
    channel->set(BinarySignal::BINARY_LOW);
  }
  bcm2835_close(); 
  }

void BCM2835Interface::setClockDivider(float frequency) {
  // Divides the basic 19.2MHz PWM clock.
  int divider = static_cast<int>(ceil(19.2 * 10e6 / frequency));
  auto fDivider = clockDividerMapping.find(divider);
  if (fDivider != clockDividerMapping.end()) {
    this->clockDivider_ = fDivider->second;
  } else {
    this->clockDivider_ = 1;
  }
}

BCM2835PWMChannel* BCM2835Interface::createChannel(
    const BCM2835PWMChannel::Builder& builder) {
  return new BCM2835PWMChannel(
      builder, std::bind(&BCM2835Interface::setClockDivider, this,
                         std::placeholders::_1));
}
BCM2835BinaryChannel* BCM2835Interface::createChannel(
    const BCM2835BinaryChannel::Builder& builder) {
  return new BCM2835BinaryChannel(builder);
}

}  // namespace communication
}  // namespace motor_controllers