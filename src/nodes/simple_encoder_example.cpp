#include <motor_controllers/communication/bcm2835_interface.h>
#include <motor_controllers/encoder/encoder.h>
#include <signal.h>

#include <chrono>
#include <cmath>
#include <ctime>
#include <iostream>
#include <thread>
#include <vector>

bool isRunning;

void onSignalReceived(int) { isRunning = false; }

void dest(motor_controllers::communication::BCM2835BinaryChannel* ptr) {
  delete ptr;
}

int main(int, char*[]) {
  using namespace motor_controllers::communication;
  using namespace motor_controllers::encoder;

  std::cout << "Connecting to BCM2835" << std::endl;
  BCM2835Interface communication = BCM2835Interface();

  BCM2835BinaryChannel::Builder p2Builder = BCM2835BinaryChannel::Builder();
  p2Builder.pinNumber = 27;
  p2Builder.channelMode = ChannelMode::EVENT_DETECT;
  p2Builder.eventDetectValue = EventDetectType::EVENT_BOTH_EDGES;
  IBinarySignalChannel::Ref p2 = communication.configureChannel(p2Builder);

  BCM2835BinaryChannel::Builder p3Builder = BCM2835BinaryChannel::Builder();
  p3Builder.pinNumber = 22;
  p3Builder.channelMode = ChannelMode::EVENT_DETECT;
  p3Builder.eventDetectValue = EventDetectType::EVENT_BOTH_EDGES;
  IBinarySignalChannel::Ref p3 = communication.configureChannel(p3Builder);

  // Create a quadrature encoder of resolution 13
  Encoder encoder(std::move(p2), std::move(p3), 13);

  communication.start();
  encoder.start(50);

  isRunning = true;
  signal(SIGINT, onSignalReceived);

  std::cout << "Starting controller" << std::endl;

  // Use async on P3
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
