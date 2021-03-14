/**
 * @file pca9685_channel.h
 * @author Pierre Venet
 * @brief
 * @version 0.1
 * @date 2021-03-05
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <motor_controllers/communication/i_pwm_signal_channel.h>

#include <functional>
#include <string>

namespace motor_controllers {

namespace communication {

/**
 * @brief The PWM channels for the PCA9685 chip.
 *
 * This chip only has PWM channels.
 *
 */
class PCA9685Channel : public IPWMSignalChannel {
 public:
  struct Builder {
    int channelId;
    float range;
  };

 public:
  PCA9685Channel(const Builder&,
                 std::function<void(uint8_t, uint8_t*)> setValue,
                 std::function<void(float)> setPWMFreq);

  ~PCA9685Channel() = default;

  PCA9685Channel(const PCA9685Channel&) = delete;

  PCA9685Channel& operator=(const PCA9685Channel&) = delete;

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

 private:
  uint8_t channel_;
  float range_;
  std::function<void(uint8_t, uint8_t*)> setValue_;
  std::function<void(float)> setPWMFreq_;
};
}  // namespace communication
}  // namespace motor_controllers
