/**
 * @file bcm2835_binary_channel.h
 * @author Pierre Venet
 * @brief
 * @version 0.1
 * @date 2021-02-28
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <motor_controllers/communication/i_binary_signal_channel.h>
#include <stdint.h>

#include <list>
#include <thread>  // std::thread

namespace motor_controllers {

namespace communication {

/**
 * @brief A channel for the BCM2835 chip
 *
 */
class BCM2835BinaryChannel : public IBinarySignalChannel {
 public:
  struct Builder {
    uint8_t pinNumber;
    ChannelMode channelMode;
    EventDetectType eventDetectValue;
  };

 public:
  /**
   * @brief Construct a new BCM2835BinaryChannel for a pin
   */
  BCM2835BinaryChannel(const Builder& builder);

  virtual ~BCM2835BinaryChannel();

  BCM2835BinaryChannel(const BCM2835BinaryChannel&) = delete;

  BCM2835BinaryChannel& operator=(const BCM2835BinaryChannel&) = delete;

 public:
  /**
   * @brief Set the value if the channel is an OUTPUT
   *
   */
  void set(const BinarySignal&) final override;

  /**
   * @brief Get the value if the channel is an INPUT or OUTPUT
   *
   * @return BinarySignal
   */
  BinarySignal get() final override;

  /**
   * @brief Get a future object with the detected event if channel is
   * EVENT_DETECT
   *
   * Can be stoped with interuptEventDetection.
   *
   * @return std::future<BinarySignal>
   */
  std::future<BinarySignal> asyncDetectEvent() final override;

  /**
   * @brief Starts a thread to check the event and call callback.
   *
   * Can be stoped with interuptEventDetection.
   *
   * @param callback
   */
  void onDetectEvent(
      const std::function<void(BinarySignal)>& callback) final override;

  /**
   * @brief Stops the thread or async waiting for event
   *
   */
  void interuptEventDetection() final override;

 public:
  /**
   * @brief Initialize the channel
   *
   * This is specific to the BCM2835 channel. Each channel must be initialized
   * individually.
   *
   */
  void initialize();

  /**
   * @brief Clean the channel
   *
   */
  void clean();

 private:
  void setInternal(const BinarySignal&);

  void setupInput();

  void setupOutput();

  void setupEventDetection(const EventDetectType&);

 private:
  const uint8_t pinNumber_;
  const EventDetectType eventDetectValue_;
  std::thread detectEventThread_;
  bool detectEventThreadAlive_;
  bool detectEventAsyncAlive_;
};
}  // namespace communication
}  // namespace motor_controllers