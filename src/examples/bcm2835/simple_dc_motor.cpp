#include <motor_controllers/communication/bcm2835/bcm2835_interface.h>
#include <signal.h>

#include <chrono>
#include <cmath>
#include <ctime>
#include <iostream>
#include <thread>
#include <vector>

bool isRunning;

void onSignalReceived(int) { isRunning = false; }

int main(int, char*[]) {
  using namespace motor_controllers::communication;

  std::cout << "Connecting to BCM2835" << std::endl;
  BCM2835Interface communication = BCM2835Interface();

  BCM2835PWMChannel::Configuration pwmABuilder = BCM2835PWMChannel::Configuration();
  pwmABuilder.pinNumber = 26;
  pwmABuilder.channel = 0;
  pwmABuilder.range = 1024;
  BCM2835PWMChannelRef pwmA = communication.configureChannel(pwmABuilder);
  pwmA->setPWMFrequency(11718.75);

  BCM2835BinaryChannel::Configuration m1Builder = BCM2835BinaryChannel::Configuration();
  m1Builder.pinNumber = 20;
  m1Builder.channelMode = ChannelMode::OUTPUT;
  BCM2835BinaryChannelRef m1 = communication.configureChannel(m1Builder);

  BCM2835BinaryChannel::Configuration m2Builder = BCM2835BinaryChannel::Configuration();
  m2Builder.pinNumber = 21;
  m2Builder.channelMode = ChannelMode::OUTPUT;
  BCM2835BinaryChannelRef m2 = communication.configureChannel(m2Builder);

  communication.start();

  isRunning = true;
  signal(SIGINT, onSignalReceived);

  std::cout << "Starting controller" << std::endl;

  m1->set(BinarySignal::BINARY_HIGH);
  m2->set(BinarySignal::BINARY_LOW);
  pwmA->setDutyCyle(0.5); // half max speed

  while (isRunning) std::this_thread::sleep_for(std::chrono::milliseconds(100));

  std::cout << "Stopping controller" << std::endl;
  communication.stop();

  return 0;
}
