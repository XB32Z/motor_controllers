#pragma once

#include <motor_controllers/communication/i_binary_signal_channel.h>
#include <motor_controllers/communication/i_pwm_signal_channel.h>
#include <motor_controllers/encoder/encoder.h>

#include <chrono>         // std::chrono
#include <string>         // std::string
#include <thread>         // std::thread
#include <unordered_map>  // std::unordered_map
#include <vector>         // std::vector

namespace motor_controllers {
namespace motor {
class DCMotor {
 public:
  typedef std::unique_ptr<DCMotor> Ref;

 public:
  struct Configuration {
    // PWM channel and its frequency
    communication::IPWMSignalChannel::Ref pwmChannel;
    double pwmFrequency = 20000;

    // Direction control channels
    std::vector<communication::IBinarySignalChannel::Ref> directionControl;
    std::vector<communication::BinarySignal> forwardConfiguration;
    std::vector<communication::BinarySignal> backwardConfiguration;
    std::vector<communication::BinarySignal> stopConfiguration;

    // Encoder and its sampling frequency
    encoder::Encoder::Ref encoder;
    double encoderSamplingFrequency = 500;

    // Motor constants
    double minDutyCycle;
    double maxSpeed;

    // PID controller constants
    double Kp = 1.0, Ki = 0.0, Kd = 0.0;
    std::chrono::microseconds dt = std::chrono::microseconds(10);
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
  void controlLoop();

  void setForward();

  void setBackward();

  void setStop();

 private:
  communication::IPWMSignalChannel::Ref pwmChannel_;
  const double pwmFrequency_;
  std::vector<communication::IBinarySignalChannel::Ref> directionControl_;
  std::vector<communication::BinarySignal> forwardConfiguration_;
  std::vector<communication::BinarySignal> backwardConfiguration_;
  std::vector<communication::BinarySignal> stopConfiguration_;

  encoder::Encoder::Ref encoder_;
  const double encoderSamplingFrequency_;

  std::thread controlThread_;
  std::mutex mtx_;
  bool isRunning_;

  const double minDutyCycle_, coefSpeedToDutyCycle_, maxSpeed_;

  double targetSpeed_;
  double previousError_;
  double integral_;
  const double Kp_, Ki_, Kd_;
  const std::chrono::microseconds dt_;
};
}  // namespace motor
}  // namespace motor_controllers
