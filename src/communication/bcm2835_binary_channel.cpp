#include <bcm2835.h>
#include <motor_controllers/communication/bcm2835_binary_channel.h>

#include <stdexcept>

namespace motor_controllers {

namespace communication {

BCM2835BinaryChannel::BCM2835BinaryChannel(const Builder& builder)
    : IBinarySignalChannel(),
      pinNumber_(builder.pinNumber),
      channelType_(builder.channelMode),
      eventDetectValue_(builder.eventDetectValue),
      detectEventThreadAlive_(false) {}

BCM2835BinaryChannel::~BCM2835BinaryChannel() {
  if (!this->isCommunicationClosed()) {
    if (this->channelType_ == ChannelMode::OUTPUT) {
      this->setInternal(BinarySignal::BINARY_LOW);
    }
  }

  if (this->detectEventThreadAlive_) {
    this->detectEventThreadAlive_ = false;
    this->detectEventThread_.join();
  }
}

void BCM2835BinaryChannel::set(const BinarySignal& value) {
  if (this->isCommunicationClosed()) {
    throw std::runtime_error(
        "BCM2835BinaryChannel: communication is closed, cannot set value");
  }
  if (this->channelType_ == ChannelMode::OUTPUT) {
    this->setInternal(value);
  } else {
    throw std::runtime_error("Cannot write on a INPUT channel");
  }
}

BinarySignal BCM2835BinaryChannel::get() {
  if (this->isCommunicationClosed()) {
    throw std::runtime_error(
        "BCM2835BinaryChannel: communication is closed, cannot get value");
  }

  // Can read whether pin is input or output
  return bcm2835_gpio_lev(this->pinNumber_) == 0 ? BinarySignal::BINARY_LOW
                                                 : BinarySignal::BINARY_HIGH;
}

std::future<BinarySignal> BCM2835BinaryChannel::asyncDetectEvent() {
  return std::async([this]() {
    while (true) {
      if (bcm2835_gpio_eds(this->pinNumber_)) {
        bcm2835_gpio_set_eds(this->pinNumber_);
        return this->eventDetectValue_;
      }
    }
  });
}

void BCM2835BinaryChannel::onDetectEvent(
    const std::function<void(void)>& callback) {
  this->detectEventThreadAlive_ = true;

  this->detectEventThread_ = std::thread([this, &callback]() {
    while (this->detectEventThreadAlive_) {
      if (bcm2835_gpio_eds(this->pinNumber_)) {
        bcm2835_gpio_set_eds(this->pinNumber_);
        callback();
      }
    }
  });
}

void BCM2835BinaryChannel::setInternal(const BinarySignal& value) {
  if (value == BinarySignal::BINARY_HIGH) {
    bcm2835_gpio_write(this->pinNumber_, HIGH);
  } else {
    bcm2835_gpio_write(this->pinNumber_, LOW);
  }
}

void BCM2835BinaryChannel::initialize() {
  if (this->channelType_ == ChannelMode::INPUT) {
    bcm2835_gpio_fsel(this->pinNumber_, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(this->pinNumber_, BCM2835_GPIO_PUD_UP);
  } else if (this->channelType_ == ChannelMode::OUTPUT) {
    bcm2835_gpio_fsel(this->pinNumber_, BCM2835_GPIO_FSEL_OUTP);
  } else if (this->channelType_ == ChannelMode::EVENT_DETECT) {
    bcm2835_gpio_fsel(this->pinNumber_, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(this->pinNumber_, BCM2835_GPIO_PUD_UP);
    if (this->eventDetectValue_ == BinarySignal::BINARY_LOW) {
      bcm2835_gpio_len(this->pinNumber_);
    } else {
      bcm2835_gpio_hen(this->pinNumber_);
    }
  }
}

}  // namespace communication
}  // namespace motor_controllers