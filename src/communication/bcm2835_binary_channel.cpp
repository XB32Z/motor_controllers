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
  return bcm2835_gpio_lev(this->pinNumber_) ? BinarySignal::BINARY_LOW
                                            : BinarySignal::BINARY_HIGH;
}

std::future<BinarySignal> BCM2835BinaryChannel::asyncDetectEvent() {
  // TODO add checks on closed communication
  return std::async([this]() {
    bcm2835_gpio_set_eds(this->pinNumber_);
    while (!bcm2835_gpio_eds(this->pinNumber_))
      ;
    return bcm2835_gpio_lev(this->pinNumber_) ? BinarySignal::BINARY_LOW
                                              : BinarySignal::BINARY_HIGH;
  });
}

void BCM2835BinaryChannel::onDetectEvent(
    const std::function<void(BinarySignal)>& callback) {
  // TODO add checks on closed communication

  if (this->detectEventThreadAlive_) {  // thread already running
    this->detectEventThreadAlive_ = false;
    this->detectEventThread_.join();
  }

  this->detectEventThreadAlive_ = true;

  this->detectEventThread_ = std::thread([this, &callback]() {
    while (this->detectEventThreadAlive_) {
      bcm2835_gpio_set_eds(this->pinNumber_);
      while (!bcm2835_gpio_eds(this->pinNumber_))
        ;
      callback(bcm2835_gpio_lev(this->pinNumber_) ? BinarySignal::BINARY_LOW
                                                  : BinarySignal::BINARY_HIGH);
    }
  });
}

void BCM2835BinaryChannel::initialize() {
  this->reset();

  if (this->channelType_ == ChannelMode::INPUT) {
    this->setupInput();
  } else if (this->channelType_ == ChannelMode::OUTPUT) {
    this->setupOutput();
  } else if (this->channelType_ == ChannelMode::EVENT_DETECT) {
    this->setupInput();
    this->setupEventDetection(this->eventDetectValue_);
  }
}

void BCM2835BinaryChannel::setInternal(const BinarySignal& value) {
  if (value == BinarySignal::BINARY_HIGH) {
    bcm2835_gpio_write(this->pinNumber_, HIGH);
  } else {
    bcm2835_gpio_write(this->pinNumber_, LOW);
  }
}

void BCM2835BinaryChannel::reset() {
  bcm2835_gpio_clr(this->pinNumber_);
  bcm2835_gpio_clr_ren(this->pinNumber_);
  bcm2835_gpio_clr_fen(this->pinNumber_);
  bcm2835_gpio_clr_hen(this->pinNumber_);
  bcm2835_gpio_clr_len(this->pinNumber_);
  bcm2835_gpio_clr_aren(this->pinNumber_);
  bcm2835_gpio_clr_afen(this->pinNumber_);
}

void BCM2835BinaryChannel::setupInput() {
  bcm2835_gpio_fsel(this->pinNumber_, BCM2835_GPIO_FSEL_INPT);
  bcm2835_gpio_set_pud(this->pinNumber_, BCM2835_GPIO_PUD_UP);
}

void BCM2835BinaryChannel::setupOutput() {
  bcm2835_gpio_fsel(this->pinNumber_, BCM2835_GPIO_FSEL_OUTP);
}

void BCM2835BinaryChannel::setupEventDetection(
    const EventDetectType& eventDetectValue) {
  switch (eventDetectValue) {
    case EventDetectType::EVENT_HIGH:
      bcm2835_gpio_hen(this->pinNumber_);
      break;
    case EventDetectType::EVENT_LOW:
      bcm2835_gpio_len(this->pinNumber_);
      break;
    case EventDetectType::EVENT_RISING_EDGE:
      bcm2835_gpio_ren(this->pinNumber_);
      break;
    case EventDetectType::EVENT_FALING_EDGE:
      bcm2835_gpio_fen(this->pinNumber_);
      break;
    case EventDetectType::EVENT_BOTH_EDGES:
      bcm2835_gpio_ren(this->pinNumber_);
      bcm2835_gpio_fen(this->pinNumber_);
      break;

    default:
      break;
  }
}
}  // namespace communication
}  // namespace motor_controllers