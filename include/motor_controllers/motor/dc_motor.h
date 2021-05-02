#pragma once

#include <motor_controllers/communication/i_binary_signal_channel.h>
#include <motor_controllers/communication/i_pwm_signal_channel.h>
#include <motor_controllers/encoder/encoder.h>

#include <string>         // std::string
#include <thread>         // std::thread
#include <unordered_map>  // std::unordered_map
#include <vector>         // std::vector

namespace motor_controllers {
namespace motor {
class DCMotor {
 public:
  struct Configuration {
    communication::IPWMSignalChannel::Ref pwmChannel;
    encoder::Encoder::Ref encoder;
    std::vector<communication::IBinarySignalChannel::Ref> directionControl;
    std::vector<communication::BinarySignal> forwardConfiguration;
    std::vector<communication::BinarySignal> backwardConfiguration;
    std::vector<communication::BinarySignal> stopConfiguration;
    double minDutyCycle;
    double maxSpeed;
    float Kp, Ki, Kd;
  };

 public:
  DCMotor(Configuration&);

  ~DCMotor();

  DCMotor(const DCMotor&) = delete;

  DCMotor& operator=(const DCMotor&) = delete;

 public:
  void start();

  void stop();

  virtual void setSpeed(double);

  virtual double getSpeed() const;

 private:
  void controlLoop(std::chrono::microseconds dt);

  void forward();

  void backward();

  void stop();

 private:
  communication::IPWMSignalChannel::Ref pwmChannel_;
  encoder::Encoder::Ref encoder_;
  std::vector<communication::IBinarySignalChannel::Ref> directionControl_;
  std::vector<communication::BinarySignal> forwardConfiguration_;
  std::vector<communication::BinarySignal> backwardConfiguration_;
  std::vector<communication::BinarySignal> stopConfiguration_;

  std::thread controlThread_;
  std::mutex mtx_;
  bool isRunning_;

  const double minDutyCycle_, coefSpeedToDutyCycle_, maxSpeed_;

  double targetSpeed_;
  double previousError_;
  double integral_;
  const float Kp_, Ki_, Kd_;

};
}  // namespace motor
}  // namespace motor_controllers
