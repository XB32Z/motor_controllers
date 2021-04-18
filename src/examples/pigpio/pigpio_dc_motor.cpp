#include <motor_controllers/communication/pigpio_interface.h>
#include <motor_controllers/encoder/encoder.h>
#include <signal.h>

#include <chrono>    // std::chrono::milliseconds
#include <iostream>  // std::cout, std::endl
#include <thread>    // std::this_thread::sleep_for

bool isRunning;

void onSignalReceived(int) { isRunning = false; }

int main(int, char*[]) {
  using namespace motor_controllers::communication;
  using namespace motor_controllers::encoder;

  std::cout << "Connecting to BCM2835 using pigpio" << std::endl;
  PiGPIOInterface communication(5);  // sample rate = 5us

  // Motor
  PiGPIOPWMChannel::Configuration pwmABuilder =
      PiGPIOPWMChannel::Configuration();
  pwmABuilder.pinNumber = 26;
  pwmABuilder.range = 1024;
  PiGPIOPWMChannelRef pwmA = communication.configureChannel(pwmABuilder);
  pwmA->setPWMFrequency(500.0);

  PiGPIOBinaryChannel::Configuration m1Builder =
      PiGPIOBinaryChannel::Configuration();
  m1Builder.pinNumber = 20;
  m1Builder.channelMode = ChannelMode::OUTPUT;
  PiGPIOBinaryChannelRef m1 = communication.configureChannel(m1Builder);

  PiGPIOBinaryChannel::Configuration m2Builder =
      PiGPIOBinaryChannel::Configuration();
  m2Builder.pinNumber = 21;
  m2Builder.channelMode = ChannelMode::OUTPUT;
  PiGPIOBinaryChannelRef m2 = communication.configureChannel(m2Builder);

  // Encoder
  PiGPIOBinaryChannel::Configuration p2Builder =
      PiGPIOBinaryChannel::Configuration();
  p2Builder.pinNumber = 27;
  p2Builder.channelMode = ChannelMode::EVENT_DETECT;
  p2Builder.eventDetectValue = EventDetectType::EVENT_BOTH_EDGES;
  IBinarySignalChannel::Ref p2 = communication.configureChannel(p2Builder);

  PiGPIOBinaryChannel::Configuration p3Builder =
      PiGPIOBinaryChannel::Configuration();
  p3Builder.pinNumber = 22;
  p3Builder.channelMode = ChannelMode::EVENT_DETECT;
  p3Builder.eventDetectValue = EventDetectType::EVENT_BOTH_EDGES;
  IBinarySignalChannel::Ref p3 = communication.configureChannel(p3Builder);

  // Create an encoder of resolution 13
  Encoder encoder(std::move(p2), 13);

  communication.start();
  encoder.start(50);

  isRunning = true;
  signal(SIGINT, onSignalReceived);

  std::cout << "Starting controller" << std::endl;

  m1->set(BinarySignal::BINARY_HIGH);
  m2->set(BinarySignal::BINARY_LOW);
  pwmA->setDutyCyle(0.5);  // half max speed

  while (isRunning) {
    float speed = encoder.getSpeed() * 60.0;
    Direction direction = encoder.getDirection();
    if (direction == Direction::BACKWARD) speed = -speed;
    if (direction != Direction::INVALID)
      std::cout << "Current speed " << speed << " RPM" << std::endl;
    else
      std::cout << "Invalid direction, corrupted data" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::cout << "Stopping controller" << std::endl;
  encoder.stop();
  communication.stop();

  return 0;
}
