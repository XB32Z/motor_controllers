/**
 * @file communication_channel.h
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
 * @brief Class to send a PWM signal
 *
 * This class is an interface. It can be implemented for each chip.
 *
 *
 */
class CommunicationChannel {
 public:
  CommunicationChannel();

  virtual ~CommunicationChannel() = default;

 public:
  /**
   * @brief Set the PWM value to send.
   *
   * @param value the value range depends on the chip used.
   */
  virtual void setValue(float value) = 0;

  /**
   * @brief Inform the channel that the communication is closed.
   *
   * This will prevent from setting the values and set an internal flag.
   */
  virtual void closeCommunication();

  /**
   * @brief Flag if the communication was already closed.
   *
   * @return true if the communication was closed
   * @return false else
   */
  virtual bool isCommunicationClosed();

 private:
  bool isCommunicationOpen_;
};
}  // namespace communication

}  // namespace motor_controllers