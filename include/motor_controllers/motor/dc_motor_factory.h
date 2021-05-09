
/**
 * @file dc_motor_factory.h
 * @author Pierre Venet
 * @brief Declaration of a helper class to construct DC motors from pins
 * directly.
 * @version 0.1
 * @date 2021-05-09
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once
#include <motor_controllers/communication/channel_builder.h>
#include <motor_controllers/communication/i_communication_interface.h>
#include <motor_controllers/motor/dc_motor.h>

#include <memory>    // std::move, std::make_unique, std::unique_ptr
#include <optional>  // std::optional
#include <vector>    // std::vector

namespace motor_controllers {
namespace motor {

/**
 * @brief Helper class to construct DCMotor from pin numbers
 *
 * As it is always the same process to instanciate DCMotors, this class wrapps
 * the code such that it is easier to use.
 *
 * @tparam CommunicationInterface
 * @tparam PWMChannelConfiguration
 * @tparam BinaryChannelConfiguration
 */
template <class CommunicationInterface, class PWMChannelConfiguration,
          class BinaryChannelConfiguration>
class DCMotorFactory {
 public:
  struct Configuration {
    // Motor PWM and direction channel configuration
    PWMChannelConfiguration pwmChannelConfiguration;
    double pwmFrequency = 20000;
    std::vector<BinaryChannelConfiguration> directionChannelsConfiguration;

    // Encoder binary channels configurations
    BinaryChannelConfiguration encoderChannelAConfiguration;
    std::optional<BinaryChannelConfiguration> encoderChannelBConfiguration;
    int encoderResolution;
    double encoderSamplingFrequency = 500;

    // Direction
    std::vector<communication::BinarySignal> forwardConfiguration;
    std::vector<communication::BinarySignal> backwardConfiguration;
    std::vector<communication::BinarySignal> stopConfiguration;

    // Motor constants
    double minDutyCycle;
    double maxSpeed;

    // Controller constants
    double Kp = 1.0, Ki = 0.0, Kd = 0.0;
    std::chrono::microseconds dt = std::chrono::microseconds(10);
  };

 public:
  DCMotorFactory(std::unique_ptr<CommunicationInterface> communicationInterface)
      : communicationInterface_(std::move(communicationInterface)) {}

  ~DCMotorFactory() = default;

  DCMotorFactory(const DCMotorFactory&) = delete;

  DCMotorFactory& operator=(const DCMotorFactory&) = delete;

 public:
  DCMotor::Ref createMotor(const Configuration& configuration) {
    DCMotor::Configuration motorConf = DCMotor::Configuration();

    // Motor channels
    motorConf.pwmChannel = this->communicationInterface_->configureChannel(
        configuration.pwmChannelConfiguration);
    motorConf.pwmFrequency = configuration.pwmFrequency;

    for (const auto& channelConf :
         configuration.directionChannelsConfiguration) {
      motorConf.directionControl.emplace_back(
          this->communicationInterface_->configureChannel(channelConf));
    }

    // Encoder
    auto encoderChannelA = this->communicationInterface_->configureChannel(
        configuration.encoderChannelAConfiguration);
    if (configuration.encoderChannelBConfiguration) {
      // Quadrature encoder
      auto encoderChannelB = this->communicationInterface_->configureChannel(
          *configuration.encoderChannelBConfiguration);
      motorConf.encoder = std::make_unique<encoder::Encoder>(
          std::move(encoderChannelA), std::move(encoderChannelB),
          configuration.encoderResolution);
    } else {
      // Single encoder
      motorConf.encoder = std::make_unique<encoder::Encoder>(
          std::move(encoderChannelA), configuration.encoderResolution);
    }

    motorConf.forwardConfiguration = configuration.forwardConfiguration;
    motorConf.backwardConfiguration = configuration.backwardConfiguration;
    motorConf.stopConfiguration = configuration.stopConfiguration;

    // Motor constants
    motorConf.minDutyCycle = configuration.minDutyCycle;
    motorConf.maxSpeed = configuration.maxSpeed;

    // Controller constants
    motorConf.Kp = configuration.Kp;
    motorConf.Ki = configuration.Ki;
    motorConf.Kd = configuration.Kd;

    return std::unique_ptr<DCMotor>(new DCMotor(motorConf));
  }

  void startCommunication() { this->communicationInterface_->start(); }

  void stopCommunication() { this->communicationInterface_->stop(); }

 private:
  std::unique_ptr<CommunicationInterface> communicationInterface_;
};
}  // namespace motor
}  // namespace motor_controllers
