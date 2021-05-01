/**
 * @file pigpio_channel_modes.h
 * @author Pierre Venet
 * @brief
 * @version 0.1
 * @date 2021-04-18
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

namespace motor_controllers {

namespace communication {

/**
 * @brief Enumeration to allow user to set alt mode on channels.
 *
 * Used for example when creating a hardware PWM channel.
 * In that case, on RPis (as known when writting this) the pins supporting
 * hardware PWM must be set to ALT5.
 *
 */
enum class PiGPIOAltMode {
  PI_INPUT = 0,
  PI_OUTPUT = 1,
  PI_ALT0 = 4,
  PI_ALT1 = 5,
  PI_ALT2 = 6,
  PI_ALT3 = 7,
  PI_ALT4 = 3,
  PI_ALT5 = 2
};
}  // namespace communication
}  // namespace motor_controllers
