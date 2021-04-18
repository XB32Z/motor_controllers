/**
 * @file pigpio_binary_channel.h
 * @author Pierre Venet
 * @brief
 * @version 0.1
 * @date 2021-04-05
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
 * @brief Binary channel wrapper for the pigpio library.
 * 
 * Allows to set, get or detect events on a pin.
 *
 */
class PiGPIOBinaryChannel : public IBinarySignalChannel {
 public:
  struct Configuration {
    uint8_t pinNumber;
    ChannelMode channelMode;
    EventDetectType eventDetectValue;
  };

 public:
  /**
   * @brief Construct a new PiGPIOBinaryChannel for a pin
   */
  PiGPIOBinaryChannel(const Configuration& builder);

  virtual ~PiGPIOBinaryChannel();

  PiGPIOBinaryChannel(const PiGPIOBinaryChannel&) = delete;

  PiGPIOBinaryChannel& operator=(const PiGPIOBinaryChannel&) = delete;

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
   * This is specific to the PIGPIO channel. Each channel must be initialized
   * individually.
   *
   */
  void initialize();

 private:
  void setInternal(const BinarySignal&);

  void setupInput();

  void setupOutput();

  void setupEventDetection();

  void onGPIOChangeState(int, int, uint32_t);

 private:
  const uint8_t pinNumber_;

  const EventDetectType eventDetectValue_;
  std::thread detectEventThread_;
  bool detectEventThreadAlive_;
  bool detectEventAsyncAlive_;

  BinarySignal eventDetectCurrentValue_;
  std::mutex eventDetectionMutex_;
  std::condition_variable eventDetectionCondVar_;
  bool eventDetectionReady_;
};
}  // namespace communication
}  // namespace motor_controllers