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
 */
class Encoder {
 public:
  /**
   * @brief Construct a quadrature Encoder
   *
   * Quadrature encoders have two channels and can read the direction of the
   * shaft as well as its velocity.
   *
   * Both channels ownership will be moved to the instance.
   *
   */
  Encoder(communication::IBinarySignalChannel::Ref channelA,
          communication::IBinarySignalChannel::Ref channelB,
          unsigned int resolution);

  /**
   * @brief Construct a new simple Encoder
   *
   * Simple encoders have only one channel and can only read the velocity of the
   * shaft. The direction returned by getDirection will always be FORWARD.
   *
   * The channel ownership will be moved to the instance.
   *
   */
  Encoder(communication::IBinarySignalChannel::Ref channel,
          unsigned int resolution);

  /**
   * @brief Destroy the Encoder object
   *
   */
  ~Encoder();

 public:
  /**
   * @brief Get the currently estimated velocity of the shaft.
   *
   * @return float in ticks per second
   */
  float getSpeed() const;

  /**
   * @brief Get the direction of the shaft for quadrature encoders.
   *
   * @return Direction
   */
  Direction getDirection() const;

  /**
   * @brief Get the count of tick since started
   *
   * @return ulong
   */
  ulong getCount() const;

 public:
  /**
   * @brief Start a thread to estimate the velocity of the shaft at the
   * specified sampling frequency.
   *
   * @param samplingFrequency
   */
  void start(float samplingFrequency);

  void stop();

 private:
  /**
   * @brief Estimate velocity and direction of a quadrature encoder.
   *
   * This version is using much CPU but has a better accuracy a lower
   * regimes.
   * 
   * TODO way to swith between them.
   *
   * @param samplingPeriod
   */
  void estimateVelocityQuadratureEncoder(
      std::chrono::microseconds samplingPeriod);

  /**
   * @brief Async estimate velocity and direction of a quadrature encoder.
   *
   * In this version, the program will halt waiting for an event either channel
   * this means that the program requires less CPU, however the speed estimation
   * is a bit twisted.
   * because we wait for the events, we can also have half r and update the cpt
   * every 2 events.
   *
   * @param samplingPeriod
   */
  void asyncEstimateVelocityQuadratureEncoder(
      std::chrono::microseconds samplingPeriod);

  void estimateVelocityEncoder(std::chrono::microseconds samplingPeriod);

 private:
  mutable std::mutex mtx_;
  bool running_;
  std::thread thread_;

  communication::IBinarySignalChannel::Ref channelA_, channelB_;

  const uint resolution_;

  ulong count_;
  float speed_;
  Direction direction_;
};

}  // namespace encoder
}  // namespace motor_controllers