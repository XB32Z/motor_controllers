#include <motor_controllers/communication/i_binary_signal_channel.h>

namespace motor_controllers {
namespace encoder {

enum class Direction { STOP = 0, FORWARD = 1, BACKWARD = 2 };

/**
 * @brief Encoder class
 *
 * An encoder is a device, attached to the shaft of a motor that can read the
 * rotations of the shaft.
 *
 * There exists different types of encoders.
 *
 */
class Encoder {
 public:
  /**
   * @brief Construct a new Encoder object
   *
   */
  Encoder(std::unique_ptr<communication::IBinarySignalChannel> channelA,
          std::unique_ptr<communication::IBinarySignalChannel> channelB);

  /**
   * @brief Destroy the Encoder object
   *
   */
  ~Encoder() = default;

 public:
  float getSpeed();

  Direction getDirection();

 private:
  std::unique_ptr<communication::IBinarySignalChannel> channelA, channelB;
};

}  // namespace encoder
}  // namespace motor_controllers