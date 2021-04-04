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
    : clockDivider_(BCM2835_PWM_CLOCK_DIVIDER_16), running_(false) {}

BCM2835Interface::~BCM2835Interface() { this->stop(); }

void BCM2835Interface::start() {
  if (this->running_) return;

  if (!bcm2835_init()) {
    throw std::runtime_error("Failed to initialize BCM2835");
  }

  bcm2835_pwm_set_clock(this->clockDivider_);

  for (auto& channel :
       this->ChannelBuilder<BCM2835PWMChannel,
                            BCM2835PWMChannel::Configuration>::channels_) {
    channel->initialize();
  }

  for (auto& channel :
       this->ChannelBuilder<BCM2835BinaryChannel,
                            BCM2835BinaryChannel::Configuration>::channels_) {
    channel->initialize();
  }

  this->running_ = true;
}

void BCM2835Interface::stop() {
  if (!this->running_) return;
  for (auto& channel :
       this->ChannelBuilder<BCM2835PWMChannel,
                            BCM2835PWMChannel::Configuration>::channels_) {
    channel->setDutyCyle(0.0);
  }

  for (auto& channel :
       this->ChannelBuilder<BCM2835BinaryChannel,
                            BCM2835BinaryChannel::Configuration>::channels_) {
    if (channel->getChannelMode() == ChannelMode::OUTPUT) {
      channel->set(BinarySignal::BINARY_LOW);
    } else if (channel->getChannelMode() == ChannelMode::EVENT_DETECT) {
      channel->interuptEventDetection();
      channel->clean();
    }
  }
  bcm2835_close();
  this->running_ = false;
}

void BCM2835Interface::setClockDivider(float frequency) {
  // Divides the basic 19.2MHz PWM clock.
  // 4.6875*10e3 is the minimum.
  unsigned int divider =
      std::ceil(std::max(19.2 * 10e6 / frequency, 4.6875 * 10e3));

  // Find next power of 2
  // https://web.archive.org/web/20160703165415/https://graphics.stanford.edu/~seander/bithacks.html#RoundUpPowerOf2
  divider--;
  divider |= divider >> 1;
  divider |= divider >> 2;
  divider |= divider >> 4;
  divider |= divider >> 8;
  divider |= divider >> 16;
  divider++;

  auto fDivider = clockDividerMapping.find(divider);
  if (fDivider != clockDividerMapping.end()) {
    this->clockDivider_ = fDivider->second;
  } else {
    this->clockDivider_ = 1;
  }
}

BCM2835PWMChannel* BCM2835Interface::createChannel(
    const BCM2835PWMChannel::Configuration& builder) {
  return new BCM2835PWMChannel(
      builder, std::bind(&BCM2835Interface::setClockDivider, this,
                         std::placeholders::_1));
}
BCM2835BinaryChannel* BCM2835Interface::createChannel(
    const BCM2835BinaryChannel::Configuration& builder) {
  return new BCM2835BinaryChannel(builder);
}

}  // namespace communication
}  // namespace motor_controllers