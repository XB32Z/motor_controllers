
/**
 * @file pca9685_interface.h
 * @author Pierre Venet
 * @brief
 * @version 0.1
 * @date 2021-02-28
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once
#include <motor_controllers/communication/channel_builder.h>
#include <motor_controllers/communication/i_communication_interface.h>
#include <motor_controllers/communication/pca9685_channel.h>

#include <map>
#include <string>

namespace motor_controllers {

namespace communication {

using PCA9685ChannelRef =
    std::unique_ptr<PCA9685Channel, std::function<void(PCA9685Channel*)>>;

/**
 * @brief Allows to open a connection from the host to a PCA9685 chip using i2c.
 *
 * Using the PCA9685Interface, a PCA9685Channel can be obtained which will
 * allow to control one of the 16 channels. This class only deals with the
 * communication, not with the data to send.
 *
 */
class PCA9685Interface
    : public ChannelBuilder<PCA9685Channel, PCA9685Channel::Builder> {
 public:
  /**
   * @brief Construct a new PCA9685Interface object
   *
   * The PCA9685Interface object allows you to connect the program to a
   * PCA985 chip via i2c. The host hardware should of course support i2c and the
   * connection and power is assumed to be done.
   *
   * @param port the i2c file to open the connection
   * @param i2cAdress the adress of the PCA9685 chip on the BUS.
   */
  PCA9685Interface(const std::string& port, int i2cAdress);

  /**
   * @brief Destroy the PCA9685Interface object
   *
   * Only when the object is destroyed, is the i2c file closed.
   *
   */
  ~PCA9685Interface();

 public:
  /**
   * @brief Re-initialize the connection to the PCA9685 chip. This must be
   * called before setting any value.
   *
   */
  void start() override;

  /**
   * @brief Stop the connection to the PCA9685 and resets the values send. It
   * does not free the i2c file.
   *
   */
  void stop() override;

  /**
   * @brief Set the frequency of the oscillator of the PCA9685
   *
   * If you connected an external clock to the PCA9685, please set the frequency
   * and the flag to true.
   *
   * @param frequency in hertz, of the oscillator.
   * @param externalClock true if you want to use an external oscillator.
   */
  void setOscillatorFrequency(float frequency, bool externalClock = false);

 private:
  void restart();

  void sleep();

  void wake();

  void setInternalClockFrequency(uint8_t prescale);

  void setExternalClockFrequency(uint8_t prescale);

  void setChannelValue(uint8_t channel, uint8_t* values);

  /**
   * @brief Set pwmFrequency_
   *
   * Note that this will be applied when start() is called.
   * The PWM frequency is shared with all channels.
   *
   * @param pwmFrequency
   */
  void setPWMFrequency(float pwmFrequency);

  /**
   * @brief Read pwmFrequency_ and set the frquency on the chip.
   *
   */
  void setFrequency();

  virtual PCA9685Channel* createChannel(
      const PCA9685Channel::Builder& channelBuilder) final override;

 private:
  int file_;
  float oscillatorFrequency_;
  float pwmFrequency_;
  bool externalClock_;
  std::map<uint8_t, PCA9685Channel*> channels_;
};
}  // namespace communication
}  // namespace motor_controllers