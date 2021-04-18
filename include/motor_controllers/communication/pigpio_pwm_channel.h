/**
 * @file pigpio_pwm_channel.h
 * @author Pierre Venet
 * @brief
 * @version 0.1
 * @date 2021-04-05
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <motor_controllers/communication/i_pwm_signal_channel.h>
#include <motor_controllers/communication/pigpio_channel_modes.h>
#include <stdint.h>

#include <functional>
#include <list>

namespace motor_controllers {

namespace communication {

/**
 * @brief A channel for the PIGPIO chip
 *
 */
class PiGPIOPWMChannel : public IPWMSignalChannel {
 public:
  struct Configuration {
    uint8_t pinNumber;
    uint32_t range;
    PiGPIOAltMode altMode_ = PiGPIOAltMode::PI_OUTPUT;
    bool isHardware = false;
  };

 public:
  /**
   * @brief Construct a new PiGPIOPWMChannel for a pin
   */
  PiGPIOPWMChannel(const Configuration& builder, uint8_t sampleRate_);

  ~PiGPIOPWMChannel();

  PiGPIOPWMChannel(const PiGPIOPWMChannel&) = delete;

  PiGPIOPWMChannel& operator=(const PiGPIOPWMChannel&) = delete;

 public:
  /**
   * @brief Set the frequency of the PWM signal.
   *
   * @param frequency in hertz
   */
  void setPWMFrequency(float frequency) final override;

  /**
   * @brief Set the Pulse Width Modulation
   *
   * The frequency of signal determines the period. The start and end of the ON
   * signal is determined by this function.
   *
   * @param start start of the signal on the period
   * @param end end of the signal on the period
   */
  void setPWM(float start, float end) final override;

  /**
   * @brief Set the Pulse Width Modulation
   *
   * The frequency of signal determines the period. The duty cycle is how much
   * percentage of the period, is the signal ON. This method is equivalent to
   * calling setPWM(0, (max-min)*dutyCycle).
   *
   * @param dutyCycle a number between 0 and 1
   */
  void setDutyCyle(float dutyCycle) final override;

  /**
   * @brief Get the minimum value that can be send as PWM signal.
   *
   * @return float
   */
  float getMinValue() const final override;

  /**
   * @brief Get the maximum value that can be send as PWM signal.
   *
   * @return float
   */
  float getMaxValue() const final override;

 public:
  /**
   * @brief Initialize the channel
   *
   * This is specific to the PIGPIO channel. Each channel must be initialized
   * individually.
   *
   */
  void initialize();

 private:
  const uint8_t pinNumber_;
  const uint32_t range_;
  const PiGPIOAltMode altMode_;
  uint32_t frequency_;
  const bool isHardware_;
  const uint8_t sampleRate_;
};
}  // namespace communication
}  // namespace motor_controllers