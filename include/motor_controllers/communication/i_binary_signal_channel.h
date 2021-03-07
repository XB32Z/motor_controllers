/**
 * @file i_binary_signal_channel.h
 * @author Pierre Venet
 * @brief
 * @version 0.1
 * @date 2021-03-05
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <motor_controllers/communication/i_signal_channel.h>

#include <future>  // std::async, std::future

namespace motor_controllers {
namespace communication {

enum class BinarySignal : bool { BINARY_LOW = false, BINARY_HIGH = true };
enum class ChannelMode { INPUT, OUTPUT, EVENT_DETECT };
/**
 * @brief Class to represent a channel used to send a signal
 *
 * This class is an interface. It can be implemented for each type of channel on
 * different chips.
 *
 * The binary channels have three modes:
 *  - INPUT the value of the channel can be read with get()
 *  - OUTPUT the value of the channel can be set with set() and read with get()
 *  - EVENT_DETECT the event type must be configured and the event can be detect
 *    asynchronously or using a thread.
 *
 */
class IBinarySignalChannel : public ISignalChannel {
 public:
  IBinarySignalChannel();

  virtual ~IBinarySignalChannel() = default;

  IBinarySignalChannel(const IBinarySignalChannel&) = delete;

  IBinarySignalChannel& operator=(const IBinarySignalChannel&) = delete;

 public:
  /**
   * @brief If the channel is an OUTPUT, sets the value on the pin.
   *
   * @param value
   */
  virtual void set(const BinarySignal& value) = 0;

  /**
   * @brief If applicable, reads the current value on the pin.
   *
   * @return BinarySignal
   */
  virtual BinarySignal get() = 0;

  /**
   * @brief Get a future object that will block untill the event is detected
   *
   * If the channel is configured on EVENT_DETECT.
   *
   */
  virtual std::future<BinarySignal> asyncDetectEvent() = 0;

  /**
   * @brief Create a thread and call the callback at every detected event
   *
   * @param callback
   */
  virtual void onDetectEvent(const std::function<void(void)>& callback) = 0;
};

}  // namespace communication

}  // namespace motor_controllers