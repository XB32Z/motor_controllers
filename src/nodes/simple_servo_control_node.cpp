#include <iostream>
#include <motor_controllers/communication/pca9685_communication.hpp>
#include <thread>

int main(int argc, char* argv[]) {
  motor_controllers::communication::PCA9685Communication communication(argv[1],
                                                                       40);

  communication.setOscillatorFrequency(27000000);
  communication.setPWMFrequency(1600);
  communication.start();

  communication.setChannelValue(0, 65535);

  std::this_thread::sleep_for(std::chrono::seconds(1));

  return 0;
}
