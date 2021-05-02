/**
 * @file bcm2835_pwm_channel.h
 * @author Pierre Venet
 * @brief
 * @version 0.1
 * @date 2021-02-28
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <motor_controllers/communication/i_pwm_signal_channel.h>
#include <stdint.h>

#include <functional>
#include <list>

namespace motor_controllers {

namespace communication {

/**
 * @brief A channel for the BCM2835 chip
 *
 */
class BCM2835PWMChannel : public IPWMSignalChannel {
 public:
  struct Configuration {
    uint8_t pinNumber;
    uint8_t channel;
    uint32_t range;
  };

 public:
  /**
   * @brief Construct a new BCM2835PWMChannel for a pin
   */
  BCM2835PWMChannel(const Configuration& builder,
                    std::function<void(float)> setPWMFreq);

  ~BCM2835PWMChannel();

  BCM2835PWMChannel(const BCM2835PWMChannel&) = delete;

  BCM2835PWMChannel& operator=(const BCM2835PWMChannel&) = delete;

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
  void setDutyCycle(float dutyCycle) final override;

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
   * This is specific to the BCM2835 channel. Each channel must be initialized
   * individually.
   *
   */
  void initialize();

 private:
  uint8_t pinNumber_;
  uint8_t pwmChannel_;
  uint32_t range_;
  std::function<void(float)> setPWMFreq_;
};
}  // namespace communication
}  // namespace motor_controllers