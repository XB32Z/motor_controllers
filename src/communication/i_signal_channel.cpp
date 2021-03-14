#include <motor_controllers/communication/i_binary_signal_channel.h>
#include <motor_controllers/communication/i_pwm_signal_channel.h>
#include <motor_controllers/communication/i_signal_channel.h>

namespace motor_controllers {

namespace communication {

ISignalChannel::ISignalChannel() : isCommunicationOpen_(true) {}

void ISignalChannel::closeCommunication() {
  this->isCommunicationOpen_ = false;
}

bool ISignalChannel::isCommunicationClosed() {
  return !this->isCommunicationOpen_;
}

IPWMSignalChannel::IPWMSignalChannel() : ISignalChannel() {}

IBinarySignalChannel::IBinarySignalChannel(const ChannelMode& mode)
    : ISignalChannel(), channelType_(mode) {}

const ChannelMode& IBinarySignalChannel::getChannelMode() const {
  return this->channelType_;
}

}  // namespace communication

}  // namespace motor_controllers