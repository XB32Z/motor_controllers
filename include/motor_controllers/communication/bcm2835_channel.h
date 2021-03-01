/**
 * @file bcm2835_channel.h
 * @author Pierre Venet
 * @brief
 * @version 0.1
 * @date 2021-02-28
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <motor_controllers/communication/communication_channel.h>

#include <list>

namespace motor_controllers {

namespace communication {

/**
 * @brief A channel for the BCM2835 chip
 *
 */
class BCM2835Channel : public CommunicationChannel {
 public:
  /**
   * @brief Construct a new BCM2835Channel for a pin
   *
   * @param pinNumber pin number (in BCM identificiation)
   * @param channel
   * @param range the range of the value
   */
  BCM2835Channel(uint8_t pinNumber, uint8_t channel, uint8_t range);

  ~BCM2835Channel() = default;

 public:
  /**
   * @brief Set the value of the pin.
   *
   * @param value
   */
  void setValue(float value) override;

 public:
  /**
   * @brief Initialize the channel
   *
   * This is specific to the BCM2835 channel. Each channel must be initialized
   * individually.
   *
   */
  void initialize();

 private:
  int pinNumber_;
  int pwmChannel_;
  int range_;
};
}  // namespace communication
}  // namespace motor_controllers