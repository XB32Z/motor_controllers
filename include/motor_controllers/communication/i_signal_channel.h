/**
 * @file i_signal_channel.h
 * @author Pierre Venet
 * @brief
 * @version 0.1
 * @date 2021-02-28
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

namespace motor_controllers {
namespace communication {

/**
 * @brief Class to represent a channel used to send a signal
 *
 * This class is an interface. It can be implemented for each type of channel on
 * different chips.
 *
 *
 */
class ISignalChannel {
 public:
  ISignalChannel();

  virtual ~ISignalChannel() = default;

 public:

  /**
   * @brief Inform the channel that the communication is closed.
   *
   * This will prevent from setting the values and set an internal flag.
   */
  void closeCommunication();

  /**
   * @brief Flag if the communication was already closed.
   *
   * @return true if the communication was closed
   * @return false else
   */
  bool isCommunicationClosed();

 private:
  bool isCommunicationOpen_;
};

}  // namespace communication

}  // namespace motor_controllers