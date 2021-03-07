#include <motor_controllers/encoder/encoder.h>

namespace motor_controllers {
namespace encoder {

Encoder::Encoder(std::unique_ptr<communication::IBinarySignalChannel> channelA,
                 std::unique_ptr<communication::IBinarySignalChannel> channelB)
    : channelA(std::move(channelA)), channelB(std::move(channelB)) {}

float Encoder::getSpeed() {}

Direction Encoder::getDirection() {}
}  // namespace encoder
}  // namespace motor_controllers