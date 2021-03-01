/**
 * @file bcm2835_communication.h
 * @author Pierre Venet
 * @brief
 * @version 0.1
 * @date 2021-02-28
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <motor_controllers/communication/bcm2835_channel.h>
#include <motor_controllers/communication/communication_channel_builder.h>

#include <functional>
#include <map>

namespace motor_controllers {

namespace communication {

/**
 * @brief Communication class with the BCM2835 chip.
 *
 * The BCM2835 is the chip that is, for example, on the RapsberryPis to control
 * the GPIOs.
 *
 * This class allows to connect to the BCM2835 and configure the pins with a
 * BCM2835Channel. This object can be used to set the pin values.
 *
 */
class BCM2835Communication : public CommunicationChannelBuilder<BCM2835Channel> {
 public:
  /**
   * @brief Construct a new BCM2835Communication object
   *
   */
  BCM2835Communication();

  /**
   * @brief Destroy the BCM2835Communication object
   *
   * Also stops the communication
   */
  ~BCM2835Communication();

 public:
  /**
   * @brief Start the communication with the BCM2835 chip.
   *
   */
  void start() override;

  /**
   * @brief Stop  the communication with the BCM2835 chip.
   *
   */
  void stop() override;

  /**
   * @brief Set the frequency of the Pulse Width Modulation signal
   *
   * The BCM2835 has an oscillator of 19.2MHz. The frequencies that can be used
   * are a fraction of this default frequency. The divider must be a power of 2
   * (from 2 to 2048). This means that the frequency set here is not necessarily
   * the one used at the end if you do not use the proper divider.
   *
   * @param frequency in hertz
   */
  void setPWMFrequency(float frequency) override;

  /**
   * @brief Get the minimum value that can be send as PWM signal.
   *
   * @return float
   */
  float getMinValue() const override;

  /**
   * @briefGet the maximum value that can be send as PWM signal.
   *
   * @return float
   */
  float getMaxValue() const override;

 private:
  virtual BCM2835Channel* createChannel(uint8_t channel) final override;

 private:
  uint8_t clockDivider_;
};
}  // namespace communication
}  // namespace motor_controllers