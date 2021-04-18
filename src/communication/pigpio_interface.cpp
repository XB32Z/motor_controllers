#include <motor_controllers/communication/pigpio_interface.h>
#include <pigpio.h>

#include <algorithm>
#include <cmath>
#include <map>

namespace motor_controllers {

namespace communication {

PiGPIOInterface::PiGPIOInterface() : running_(false), sampleRate_(5) {}

PiGPIOInterface::PiGPIOInterface(uint8_t sampleRate)
    : running_(false), sampleRate_(sampleRate) {}

PiGPIOInterface::~PiGPIOInterface() { this->stop(); }

void PiGPIOInterface::start() {
  if (this->running_) return;

  if (gpioInitialise() < 0) {
    throw std::runtime_error("Failed to initialize PiGPIO");
  }

  for (auto& channel :
       this->ChannelBuilder<PiGPIOPWMChannel,
                            PiGPIOPWMChannel::Configuration>::channels_) {
    channel->initialize();
  }

  for (auto& channel :
       this->ChannelBuilder<PiGPIOBinaryChannel,
                            PiGPIOBinaryChannel::Configuration>::channels_) {
    channel->initialize();
  }

  this->running_ = true;
}

void PiGPIOInterface::stop() {
  if (!this->running_) return;
  for (auto& channel :
       this->ChannelBuilder<PiGPIOPWMChannel,
                            PiGPIOPWMChannel::Configuration>::channels_) {
    channel->setDutyCyle(0.0);
  }

  for (auto& channel :
       this->ChannelBuilder<PiGPIOBinaryChannel,
                            PiGPIOBinaryChannel::Configuration>::channels_) {
    if (channel->getChannelMode() == ChannelMode::OUTPUT) {
      channel->set(BinarySignal::BINARY_LOW);
    } else if (channel->getChannelMode() == ChannelMode::EVENT_DETECT) {
      channel->interuptEventDetection();
    }
  }
  gpioTerminate();
  this->running_ = false;
}

PiGPIOPWMChannel* PiGPIOInterface::createChannel(
    const PiGPIOPWMChannel::Configuration& builder) {
  return new PiGPIOPWMChannel(builder, this->sampleRate_);
}
PiGPIOBinaryChannel* PiGPIOInterface::createChannel(
    const PiGPIOBinaryChannel::Configuration& builder) {
  return new PiGPIOBinaryChannel(builder);
}

}  // namespace communication
}  // namespace motor_controllers