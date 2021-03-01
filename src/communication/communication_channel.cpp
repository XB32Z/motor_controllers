#include <motor_controllers/communication/communication_channel.h>

namespace motor_controllers {

namespace communication {

CommunicationChannel::CommunicationChannel() : isCommunicationOpen_(true) {}

void CommunicationChannel::closeCommunication() {
  this->isCommunicationOpen_ = false;
}

bool CommunicationChannel::isCommunicationClosed() {
  return !this->isCommunicationOpen_;
}
}  // namespace communication

}  // namespace motor_controllers