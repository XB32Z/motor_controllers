/**
 * @file encoder.h
 * @author Pierre Venet
 * @brief
 * @version 0.1
 * @date 2021-03-11
 *
 * @copyright Copyright (c) 2021
 *
 */
#include <motor_controllers/communication/i_binary_signal_channel.h>

#include <mutex>
#include <thread>

namespace motor_controllers {
namespace encoder {

enum class Direction { STOP = 0, FORWARD = 1, BACKWARD = 2, INVALID = 3 };

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
  float getSpeed() const;

  Direction getDirection() const;

 public:
  void start();

  void stop();

 private:
  void measureSpeed();

 private:
  std::unique_ptr<communication::IBinarySignalChannel> channelA, channelB;
  mutable std::mutex mtx_;
  bool running_;
  std::thread thread_;
  float speed_;
  Direction direction_;
};

}  // namespace encoder
}  // namespace motor_controllers