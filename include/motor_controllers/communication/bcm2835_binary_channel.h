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
    BinarySignal eventDetectValue;
  };

 public:
  /**
   * @brief Construct a new BCM2835BinaryChannel for a pin
   */
  BCM2835BinaryChannel(const Builder& builder);

  ~BCM2835BinaryChannel();

  BCM2835BinaryChannel(const BCM2835BinaryChannel&) = delete;

  BCM2835BinaryChannel& operator=(const BCM2835BinaryChannel&) = delete;

 public:
  /**
   * @brief Set the value if the channel is an OUTPUT
   *
   */
  virtual void set(const BinarySignal&) final override;

  /**
   * @brief Get the value if the channel is an INPUT or OUTPUT
   *
   * @return BinarySignal
   */
  virtual BinarySignal get() final override;

  /**
   * @brief Get a future object with the detected event if channel is
   * EVENT_DETECT
   *
   * @return std::future<bool>
   */
  virtual std::future<BinarySignal> asyncDetectEvent() final override;

  /**
   * @brief Starts a thread to check the event and call callback
   *
   * @param callback
   */
  virtual void onDetectEvent(
      const std::function<void(void)>& callback) final override;

 public:
  /**
   * @brief Initialize the channel
   *
   * This is specific to the BCM2835 channel. Each channel must be initialized
   * individually.
   *
   */
  void initialize();

  void setInternal(const BinarySignal&);

 private:
  const uint8_t pinNumber_;
  const ChannelMode channelType_;
  const BinarySignal eventDetectValue_;
  std::thread detectEventThread_;
  bool detectEventThreadAlive_;
};
}  // namespace communication
}  // namespace motor_controllers