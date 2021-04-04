#include <motor_controllers/communication/pca9685_interface.h>
#include <signal.h>

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

bool isRunning;

void onSignalReceived(int) { isRunning = false; }

int main(int argc, char* argv[]) {
  std::string fileName = "/dev/i2c-1";
  uint8_t i2cAdress = 0x40;

  if (argc > 1) {
    fileName = std::string(argv[1]);
  }
  if (argc > 2) {
    i2cAdress = std::stoi(argv[2]);
  }

  using namespace motor_controllers::communication;

  std::cout << "Connecting to " << fileName << " at " << i2cAdress << std::endl;
  PCA9685Interface communication(fileName, i2cAdress);
  communication.setOscillatorFrequency(27000000);

  // Create the 16 channels of the PCA9685 chip
  std::vector<PCA9685ChannelRef> channels;
  for (uint8_t i = 0; i < 16; ++i) {
    PCA9685Channel::Configuration builder = {.channelId = i, .range = 0x0FFF};
    PCA9685ChannelRef channel = communication.configureChannel(builder);
    channel->setPWMFrequency(50);  // Freq is shared with all channels.
    channels.push_back(std::move(channel));
  }

  communication.start();

  isRunning = true;
  signal(SIGINT, onSignalReceived);

  std::cout << "Starting controller" << std::endl;

  for (auto& channel : channels) {
    if (!isRunning) break;
    for (uint16_t pulselen = 150; pulselen < 600; ++pulselen) {
      channel->setDutyCyle(pulselen / channel->getMaxValue());
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  for (auto& channel : channels) {
    if (!isRunning) break;
    for (uint16_t pulselen = 600; pulselen < 150; --pulselen) {
      channel->setDutyCyle(pulselen / channel->getMaxValue());
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  std::cout << "Stopping controller" << std::endl;
  communication.stop();

  return 0;
}
