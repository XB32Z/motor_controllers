/**
 * @file bcm2835_interface.h
 * @author Pierre Venet
 * @brief
 * @version 0.1
 * @date 2021-02-28
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <motor_controllers/communication/bcm2835_binary_channel.h>
#include <motor_controllers/communication/bcm2835_pwm_channel.h>
#include <motor_controllers/communication/channel_builder.h>

#include <functional>
#include <map>

namespace motor_controllers {

namespace communication {
using BCM2835PWMChannelRef =
    std::unique_ptr<BCM2835PWMChannel, std::function<void(ISignalChannel*)>>;
using BCM2835BinaryChannelRef =
    std::unique_ptr<BCM2835BinaryChannel, std::function<void(ISignalChannel*)>>;

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
class BCM2835Interface
    : public ChannelBuilder<BCM2835PWMChannel, BCM2835PWMChannel::Builder>,
      public ChannelBuilder<BCM2835BinaryChannel,
                            BCM2835BinaryChannel::Builder> {
 public:
  /**
   * @brief Construct a new BCM2835Interface object
   *
   */
  BCM2835Interface();

  /**
   * @brief Destroy the BCM2835Interface object
   *
   * Also stops the communication
   */
  ~BCM2835Interface();

 public:
  using ChannelBuilder<BCM2835BinaryChannel,
                       BCM2835BinaryChannel::Builder>::configureChannel;
  using ChannelBuilder<BCM2835PWMChannel,
                       BCM2835PWMChannel::Builder>::configureChannel;
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

 private:
  void setClockDivider(float frequency);

  BCM2835PWMChannel* createChannel(
      const BCM2835PWMChannel::Builder& channel) final override;

  BCM2835BinaryChannel* createChannel(
      const BCM2835BinaryChannel::Builder& channel) final override;

 private:
  uint8_t clockDivider_;
  bool running_;
};
}  // namespace communication
}  // namespace motor_controllers