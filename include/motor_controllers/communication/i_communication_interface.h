/**
 * @file i_communication_interface.h
 * @author Pierre Venet
 * @brief
 * @version 0.1
 * @date 2021-02-28
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <motor_controllers/communication/i_signal_channel.h>

#include <functional>
#include <memory>

namespace motor_controllers {
namespace communication {

/**
 * @brief Interface to implement a communication interface to chips.
 *
 * The chips can in turn be used to control motor with Pulse Width Modulation.
 * The classes which implement this interface only deal with the communication
 * to a given chip.
 *
 * Each chip can have one or more channels of heterogenous types.
 * This class alone does not allow to send data to the channels of the chip. For
 * that, use the CommunicationChannelBuilder pattern which will allow you to
 * create subclasses of ISignalChannel which, in turn, can be used to send
 * differnt types of signal to each channel of the chip.
 *
 */
class ICommunicationInterface {
 public:
  typedef std::unique_ptr<ICommunicationInterface> Ref;

 public:
  virtual ~ICommunicationInterface() = default;

 public:
  /**
   * @brief Start the communication with the chips.
   *
   * Must be called before operating the channels.
   *
   */
  virtual void start() = 0;

  /**
   * @brief Stop the communication with the chips.
   *
   */
  virtual void stop() = 0;
};
}  // namespace communication

}  // namespace motor_controllers