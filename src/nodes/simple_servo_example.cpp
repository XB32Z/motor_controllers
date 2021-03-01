#include <signal.h>

#include <ctime>
#include <iostream>
#include <cmath>
#include <motor_controllers/communication/pca9685_communication.h>
#include <thread>

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

  std::cout << "Connecting to " << fileName << " at " << i2cAdress << std::endl;
  motor_controllers::communication::PCA9685Communication communication(
      fileName, i2cAdress);

  communication.setOscillatorFrequency(27000000);
  communication.setPWMFrequency(1600);
  auto channel0 = communication.configureChannel(0);
  communication.start();

  signal(SIGINT, onSignalReceived);

  isRunning = true;

  std::cout << "Starting controller" << std::endl;
  float amplitude = communication.getMaxValue() - communication.getMinValue();
  float frequency = 5;
  clock_t begin_time = clock();
  while (isRunning) {
    channel0->setValue(amplitude * sin(2 * M_PI * frequency *
                                      (clock() - begin_time) / CLOCKS_PER_SEC));
    begin_time = clock();
  }

  std::cout << "Stopping controller" << std::endl;
  communication.stop();

  return 0;
}
