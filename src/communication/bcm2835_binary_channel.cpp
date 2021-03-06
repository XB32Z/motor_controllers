#include <bcm2835.h>
#include <motor_controllers/communication/bcm2835_binary_channel.h>

#include <stdexcept> 

namespace motor_controllers {

namespace communication {

BCM2835BinaryChannel::BCM2835BinaryChannel(const Builder& builder)
    : IBinarySignalChannel(),
      pinNumber_(builder.pinNumber),
      channelType_(builder.channelType) {}

void BCM2835BinaryChannel::set(const BinarySignal& value) {
  if (this->channelType_ == ChannelType::OUTPUT) {
    if (value == BinarySignal::BINARY_HIGH) {
      bcm2835_gpio_write(this->pinNumber_, HIGH);
    } else {
      bcm2835_gpio_write(this->pinNumber_, LOW);
    }
  } else {
    throw std::runtime_error("Cannot write on a INPUT channel");
  }
}

BinarySignal BCM2835BinaryChannel::get() {
  if (this->channelType_ == ChannelType::INPUT) {
    return bcm2835_gpio_lev(this->pinNumber_) == 0 ? BinarySignal::BINARY_LOW : BinarySignal::BINARY_HIGH;
  } else {
    throw std::runtime_error("Cannot write on a INPUT channel");
  }
}

void BCM2835BinaryChannel::initialize() {
  if (this->channelType_ == ChannelType::INPUT) {
    bcm2835_gpio_fsel(this->pinNumber_, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(this->pinNumber_, BCM2835_GPIO_PUD_UP);
  } else {
    bcm2835_gpio_fsel(this->pinNumber_, BCM2835_GPIO_FSEL_OUTP);
  }
}

}  // namespace communication
}  // namespace motor_controllers