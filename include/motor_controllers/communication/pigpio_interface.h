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
 * The pigpio has the advantage over BCM2835 library to provide a software PWM
 * channel implementation. This means that any pin on a BCM2835 (or higher) chip
 * can be used to provide a PWM signal. Of course, software PWM is using much
 * more CPU than hardware.
 *
 * See: http://abyz.me.uk/rpi/pigpio/cif.html#
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
   * This default constructor allows to initialize the pigpio library and to
   * create channels. The default sample rate of 5us is used (see
   * documentation).
   */
  PiGPIOInterface();

  /**
   * @brief Construct a new PiGPIOInterface object
   *
   * This constructor allows to specify another sample rate in [1, 2, 4, 5, 8,
   * 10] us. See documentation of pigpio for pros and cons.
   *
   * @param sampleRate
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
   * @brief Start the communication through pigpio library.
   *
   */
  void start() override;

  /**
   * @brief Stop  the communication through pigpio library.
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
