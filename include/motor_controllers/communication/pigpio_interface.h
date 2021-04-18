/**
 * @file pigpio_interface.h
 * @author Pierre Venet
 * @brief
 * @version 0.1
 * @date 2021-04-05
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <motor_controllers/communication/channel_builder.h>
#include <motor_controllers/communication/pigpio_binary_channel.h>
#include <motor_controllers/communication/pigpio_pwm_channel.h>

#include <functional>
#include <map>

namespace motor_controllers {

namespace communication {
using PiGPIOPWMChannelRef =
    std::unique_ptr<PiGPIOPWMChannel, std::function<void(ISignalChannel*)>>;
using PiGPIOBinaryChannelRef =
    std::unique_ptr<PiGPIOBinaryChannel, std::function<void(ISignalChannel*)>>;

/**
 * @brief Communication class that wraps the pigpio library.
 *
 *
 */
class PiGPIOInterface
    : public ChannelBuilder<PiGPIOPWMChannel, PiGPIOPWMChannel::Configuration>,
      public ChannelBuilder<PiGPIOBinaryChannel,
                            PiGPIOBinaryChannel::Configuration> {
 public:
  /**
   * @brief Construct a new PiGPIOInterface object
   *
   */
  PiGPIOInterface();

  /**
   * @brief Construct a new PiGPIOInterface object
   *
   */
  PiGPIOInterface(uint8_t sampleRate);

  /**
   * @brief Destroy the PiGPIOInterface object
   *
   * Also stops the communication
   */
  ~PiGPIOInterface();

 public:
  using ChannelBuilder<PiGPIOBinaryChannel,
                       PiGPIOBinaryChannel::Configuration>::configureChannel;
  using ChannelBuilder<PiGPIOPWMChannel,
                       PiGPIOPWMChannel::Configuration>::configureChannel;
  /**
   * @brief Start the communication through the PIGPIO.
   *
   */
  void start() override;

  /**
   * @brief Stop  the communication with the PIGPIO chip.
   *
   */
  void stop() override;

 private:
  PiGPIOPWMChannel* createChannel(
      const PiGPIOPWMChannel::Configuration& channel) final override;

  PiGPIOBinaryChannel* createChannel(
      const PiGPIOBinaryChannel::Configuration& channel) final override;

 private:
  bool running_;
  uint8_t sampleRate_;
};
}  // namespace communication
}  // namespace motor_controllers