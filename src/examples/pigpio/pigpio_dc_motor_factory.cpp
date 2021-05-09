#include <motor_controllers/communication/pigpio/pigpio_interface.h>
#include <motor_controllers/motor/dc_motor_factory.h>
#include <signal.h>

#include <chrono>    // std::chrono::milliseconds
#include <cmath>     // std::cos
#include <iostream>  // std::cout, std::endl
#include <optional>  // std::optional
#include <thread>    // std::this_thread::sleep_for

bool isRunning;

void onSignalReceived(int) { isRunning = false; }

int main(int, char*[]) {
  using namespace motor_controllers::communication;
  using namespace motor_controllers::motor;

  typedef DCMotorFactory<PiGPIOInterface, PiGPIOPWMChannel::Configuration,
                         PiGPIOBinaryChannel::Configuration>
      PiGPIODCMotorFactory;

  std::cout << "Connecting to BCM2835 using pigpio" << std::endl;
  PiGPIODCMotorFactory factory(std::make_unique<PiGPIOInterface>(2));

  // Motor 1
  auto conf1 = PiGPIODCMotorFactory::Configuration();
  {
    conf1.pwmChannelConfiguration.pinNumber = 12;
    conf1.pwmChannelConfiguration.range = 1024;
    conf1.pwmFrequency = 20000.0;

    {
      PiGPIOBinaryChannel::Configuration m1 =
          PiGPIOBinaryChannel::Configuration();
      m1.pinNumber = 6;
      m1.channelMode = ChannelMode::OUTPUT;

      PiGPIOBinaryChannel::Configuration m2 =
          PiGPIOBinaryChannel::Configuration();
      m2.pinNumber = 13;
      m2.channelMode = ChannelMode::OUTPUT;
      conf1.directionChannelsConfiguration.push_back(m1);
      conf1.directionChannelsConfiguration.push_back(m2);
    }
    // Encoder binary channels configurations
    {
      conf1.encoderChannelAConfiguration = {
          .pinNumber = 27,
          .channelMode = ChannelMode::EVENT_DETECT,
          .eventDetectValue = EventDetectType::EVENT_BOTH_EDGES};

      PiGPIOBinaryChannel::Configuration confB =
          PiGPIOBinaryChannel::Configuration();
      confB.pinNumber = 22;
      confB.channelMode = ChannelMode::EVENT_DETECT;
      confB.eventDetectValue = EventDetectType::EVENT_BOTH_EDGES;
      conf1.encoderChannelBConfiguration =
          std::optional<PiGPIOBinaryChannel::Configuration>(confB);
      conf1.encoderResolution = 13;
      conf1.encoderSamplingFrequency = 5000;
    }

    // Direction
    conf1.forwardConfiguration = {BinarySignal::BINARY_HIGH,
                                  BinarySignal::BINARY_LOW};
    conf1.backwardConfiguration = {BinarySignal::BINARY_LOW,
                                   BinarySignal::BINARY_HIGH};
    conf1.stopConfiguration = {BinarySignal::BINARY_LOW,
                               BinarySignal::BINARY_LOW};

    // Motor constants
    conf1.minDutyCycle = 0.5;
    conf1.maxSpeed = 8000.0 / 60.0;  // rotation per seconds

    // Controller constants
    conf1.Kp = 1.0;
    conf1.Ki = 0.0;
    conf1.Kd = 0.0;
    conf1.dt = std::chrono::microseconds(10);
  }
  auto motor1 = factory.createMotor(conf1);

  // Motor 2
  auto conf2 = PiGPIODCMotorFactory::Configuration();
  {
    conf2.pwmChannelConfiguration.pinNumber = 26;
    conf2.pwmChannelConfiguration.range = 1024;
    conf2.pwmFrequency = 20000.0;

    {
      PiGPIOBinaryChannel::Configuration m1 =
          PiGPIOBinaryChannel::Configuration();
      m1.pinNumber = 20;
      m1.channelMode = ChannelMode::OUTPUT;

      PiGPIOBinaryChannel::Configuration m2 =
          PiGPIOBinaryChannel::Configuration();
      m2.pinNumber = 21;
      m2.channelMode = ChannelMode::OUTPUT;
      conf2.directionChannelsConfiguration.push_back(m1);
      conf2.directionChannelsConfiguration.push_back(m2);
    }
    // Encoder binary channels configurations
    {
      conf2.encoderChannelAConfiguration = {
          .pinNumber = 16,
          .channelMode = ChannelMode::EVENT_DETECT,
          .eventDetectValue = EventDetectType::EVENT_BOTH_EDGES};

      PiGPIOBinaryChannel::Configuration confB =
          PiGPIOBinaryChannel::Configuration();
      confB.pinNumber = 19;
      confB.channelMode = ChannelMode::EVENT_DETECT;
      confB.eventDetectValue = EventDetectType::EVENT_BOTH_EDGES;
      conf2.encoderChannelBConfiguration =
          std::optional<PiGPIOBinaryChannel::Configuration>(confB);
      conf2.encoderResolution = 13;
      conf2.encoderSamplingFrequency = 5000;
    }

    // Direction
    conf2.forwardConfiguration = {BinarySignal::BINARY_HIGH,
                                  BinarySignal::BINARY_LOW};
    conf2.backwardConfiguration = {BinarySignal::BINARY_LOW,
                                   BinarySignal::BINARY_HIGH};
    conf2.stopConfiguration = {BinarySignal::BINARY_LOW,
                               BinarySignal::BINARY_LOW};

    // Motor constants
    conf2.minDutyCycle = 0.5;
    conf2.maxSpeed = 8000.0 / 60.0;  // rotation per seconds

    // Controller constants
    conf2.Kp = 1.0;
    conf2.Ki = 0.0;
    conf2.Kd = 0.0;
    conf2.dt = std::chrono::microseconds(10);
  }
  auto motor2 = factory.createMotor(conf2);

  factory.startCommunication();
  motor1->start();
  motor2->start();

  isRunning = true;
  signal(SIGINT, onSignalReceived);

  std::cout << "Starting controller" << std::endl;

  const std::chrono::time_point<std::chrono::high_resolution_clock> startTime =
      std::chrono::high_resolution_clock::now();
  const float frequency = 1.0 / 10.0;
  const float ratio = 2.0 * M_PI * frequency * 1e-6;

  while (isRunning) {
    float speed = motor1->getSpeed() * 60.0;
    std::cout << "Current speed " << speed << " RPM" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    const auto dt = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now() - startTime);

    const auto factor =
        (1.0 + std::sin(-M_PI / 2.0 + ratio * dt.count())) / 2.0;
    motor1->setSpeed(conf1.maxSpeed * factor);
    motor2->setSpeed(conf2.maxSpeed * factor);
  }

  std::cout << "Stopping controller" << std::endl;
  motor1->stop();
  factory.stopCommunication();

  return 0;
}
