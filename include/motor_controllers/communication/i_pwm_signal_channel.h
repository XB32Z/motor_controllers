/**
 * @file i_pwm_signal_channel.h
 * @author Pierre Venet
 * @brief
 * @version 0.1
 * @date 2021-03-05
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <motor_controllers/communication/i_signal_channel.h>

namespace motor_controllers {
namespace communication {

/**
 * @brief Class to represent a channel used to send a PWM signal
 *
 *
 */
class IPWMSignalChannel : public ISignalChannel {
 public:
  IPWMSignalChannel();

  virtual ~IPWMSignalChannel() = default;

  IPWMSignalChannel(const IPWMSignalChannel&) = delete;

  IPWMSignalChannel& operator=(const IPWMSignalChannel&) = delete;

 public:
  /**
   * @brief Set the frequency of the PWM signal.
   *
   * @param frequency in hertz
   */
  virtual void setPWMFrequency(float frequency) = 0;

  /**
   * @brief Set the Pulse Width Modulation
   *
   * The frequency of signal determines the period. The start and end of the ON
   * signal is determined by this function.
   *
   * @param start start of the signal on the period
   * @param end end of the signal on the period
   */
  virtual void setPWM(float start, float end) = 0;

  /**
   * @brief Set the Pulse Width Modulation
   *
   * The frequency of signal determines the period. The duty cycle is how much
   * percentage of the period, is the signal ON. This method is equivalent to
   * calling setPWM(0, (max-min)*dutyCycle).
   *
   * @param dutyCycle a number between 0 and 1
   */
  virtual void setDutyCyle(float dutyCycle) = 0;

  /**
   * @brief Get the minimum value that can be send as PWM signal.
   *
   * @return float
   */
  virtual float getMinValue() const = 0;

  /**
   * @brief Get the maximum value that can be send as PWM signal.
   *
   * @return float
   */
  virtual float getMaxValue() const = 0;
};

}  // namespace communication

}  // namespace motor_controllers