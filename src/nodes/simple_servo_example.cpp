#include <signal.h>

#include <iostream>
#include <motor_controllers/communication/pca9685_communication.hpp>
#include <thread>

bool isRunning;

void onSignalReceived(int signum) { isRunning = false; }

int main(int argc, char* argv[]) {
  std::string fileName = "/dev/i2c-1";
  uint8_t i2cAdress = 0x40;

  if (argc > 1) {
    fileName = std::string(argv[1]);
  }
  if (argc > 2) {
    i2cAdress = std::stoi(argv[2]);
  }

  std::cout << "Connecting to " << fileName << " at " << i2cAdress << std::endl;
  motor_controllers::communication::PCA9685Communication communication(
      fileName, i2cAdress);

  communication.setOscillatorFrequency(27000000);
  communication.setPWMFrequency(1600);
  communication.start();

  isRunning = true;

  std::cout << "Starting controller" << std::endl;
  while (isRunning) {
    communication.setChannelValue(0, 65535);
  }

  std::cout << "Stopping controller" << std::endl;
  communication.stop();

  return 0;
}
