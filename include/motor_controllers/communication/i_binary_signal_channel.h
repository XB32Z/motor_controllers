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

namespace motor_controllers {
namespace communication {

enum  class BinarySignal { BINARY_LOW = 0, BINARY_HIGH = 1 };
enum class ChannelType { INPUT, OUTPUT };
/**
 * @brief Class to represent a channel used to send a signal
 *
 * This class is an interface. It can be implemented for each type of channel on
 * different chips.
 *
 *
 */
class IBinarySignalChannel : public ISignalChannel {
 public:
  IBinarySignalChannel();

  virtual ~IBinarySignalChannel() = default;

  IBinarySignalChannel(const IBinarySignalChannel&) = delete;
  
  IBinarySignalChannel& operator=(const IBinarySignalChannel&) = delete;

 public:
  virtual void set(const BinarySignal&) = 0;
  virtual BinarySignal get() = 0;
};

}  // namespace communication

}  // namespace motor_controllers