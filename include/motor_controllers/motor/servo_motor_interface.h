#pragma once
#include <motor_controllers/motor/motor_interface.h>

namespace motor_controllers {
namespace motor {
class ServoMotorInterface {
 public:
  virtual void setAngle(float) = 0;
};
}  // namespace motor
}  // namespace motor_controllers
