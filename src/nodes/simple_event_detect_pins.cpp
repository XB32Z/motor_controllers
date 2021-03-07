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

  BCM2835BinaryChannel::Builder p2Builder = BCM2835BinaryChannel::Builder();
  p2Builder.pinNumber = 27;
  p2Builder.channelMode = ChannelMode::EVENT_DETECT;
  p2Builder.eventDetectValue = BinarySignal::BINARY_HIGH;
  BCM2835BinaryChannelRef p2 = communication.configureChannel(p2Builder);

  BCM2835BinaryChannel::Builder p3Builder = BCM2835BinaryChannel::Builder();
  p3Builder.pinNumber = 22;
  p3Builder.channelMode = ChannelMode::EVENT_DETECT;
  p3Builder.eventDetectValue = BinarySignal::BINARY_HIGH;
  BCM2835BinaryChannelRef p3 = communication.configureChannel(p3Builder);

  communication.start();

  isRunning = true;
  signal(SIGINT, onSignalReceived);

  std::cout << "Starting controller" << std::endl;

  // Start a thread on P2 and print the value
  p2->onDetectEvent([]() { std::cout << "Detect event on P2" << std::endl; });

  // Use async on P3
  while (isRunning) {
    auto fut3 = p3->asyncDetectEvent();
    fut3.get();
    std::cout << "Detected event on P3" << std::endl;
  }

  std::cout << "Stopping controller" << std::endl;
  communication.stop();

  return 0;
}
