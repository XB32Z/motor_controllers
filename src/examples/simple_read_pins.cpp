#include <motor_controllers/communication/bcm2835_interface.h>
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

  BCM2835BinaryChannel::Configuration p2Builder = BCM2835BinaryChannel::Configuration();
  p2Builder.pinNumber = 27;
  p2Builder.channelMode = ChannelMode::INPUT;
  BCM2835BinaryChannelRef p2 = communication.configureChannel(p2Builder);

  BCM2835BinaryChannel::Configuration p3Builder = BCM2835BinaryChannel::Configuration();
  p3Builder.pinNumber = 22;
  p3Builder.channelMode = ChannelMode::INPUT;
  BCM2835BinaryChannelRef p3 = communication.configureChannel(p3Builder);

  communication.start();

  isRunning = true;
  signal(SIGINT, onSignalReceived);

  std::cout << "Starting controller" << std::endl;

  while (isRunning) {
    std::cout << "Reading "
              << (p2->get() == BinarySignal::BINARY_HIGH ? "HIGH" : "LOW")
              << " on P2" << std::endl;
    std::cout << "Reading "
              << (p3->get() == BinarySignal::BINARY_HIGH ? "HIGH" : "LOW")
              << " on P3" << std::endl;
  }

  std::cout << "Stopping controller" << std::endl;
  communication.stop();

  return 0;
}
