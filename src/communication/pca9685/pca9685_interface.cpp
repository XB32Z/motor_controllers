
#include <fcntl.h>
#include <unistd.h>
extern "C" {
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
}
#include <motor_controllers/communication/pca9685/pca9685_interface.h>
#include <sys/ioctl.h>

#include <algorithm>
#include <chrono>
#include <thread>

/*
 * Register definitions
 * From https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf
 * Ordered linearly
 */
#define MODE1 0X00
#define MODE2 0X01
#define SUBADR1 0X02
#define SUBADR2 0X03
#define SUBADR3 0X04
#define ALLCALLADR 0X05

// Using LED for motor control. Each LED has 4 register adresses
#define CHANNEL_0 0x06
#define CHANNEL_1 0x0A
#define CHANNEL_2 0x0E
#define CHANNEL_3 0x12
#define CHANNEL_4 0x16
#define CHANNEL_5 0x1A
#define CHANNEL_6 0x1E
#define CHANNEL_7 0x22
#define CHANNEL_8 0x26
#define CHANNEL_9 0x2A
#define CHANNEL_10 0x2E
#define CHANNEL_11 0x32
#define CHANNEL_12 0x36
#define CHANNEL_13 0x3A
#define CHANNEL_14 0x3E
#define CHANNEL_15 0x42

// Unused adresses
// 0x45 -> 0xEF

// Controll of all channels
#define CHANNEL_ALL 0xFA

// Prescaler for PWM output freq
#define PRE_SCALE 0xFE

// Enter test mode
#define TESTMODE 0xFF

// MODE1 bits
#define MODE1_RESTART 7
#define MODE1_RESTART_VAL 128
#define MODE1_EXTCLK 6
#define MODE1_EXTCLK_VAL 64
#define MODE1_AI 5
#define MODE1_AI_VAL 32
#define MODE1_SLEEP 4
#define MODE1_SLEEP_VAL 16
#define MODE1_SUB1 3
#define MODE1_SUB1_VAL 8
#define MODE1_SUB2 2
#define MODE1_SUB2_VAL 4
#define MODE1_SUB3 1
#define MODE1_SUB3_VAL 2
#define MODE1_ALLCALL 0
#define MODE1_ALLCALL_VAL 0

// MODE2 bits
// 7 -> 5 reserved
#define MODE2_INVRT 4
#define MODE2_INVRT_VAL 16
#define MODE2_OCH 3
#define MODE2_OCH_VAL 8
#define MODE2_OUTDRV 2
#define MODE2_OUTDRV_VAL 4
#define MODE2_OUTNE1 1
#define MODE2_OUTNE1_VAL 2
#define MODE2_OUTNE0 0
#define MODE2_OUTNE0_VAL 0

namespace motor_controllers {

namespace communication {

PCA9685Interface::PCA9685Interface(const std::string& port, int i2cAdress)
    : oscillatorFrequency_(2.7 * 10e6),
      pwmFrequency_(3600.f),
      externalClock_(false) {
  this->file_ = open(port.c_str(), O_RDWR);
  if (ioctl(this->file_, I2C_SLAVE, i2cAdress) < 0) {
    throw std::runtime_error("Cannot connect to " + port + " at " +
                             std::to_string(i2cAdress));
  }
}

PCA9685Interface::~PCA9685Interface() {
  for (auto& channel : this->channels_) {
    channel.second->closeCommunication();
  }
  close(this->file_);
}

void PCA9685Interface::start() {
  this->setFrequency();
  //  set it upon start!
  this->restart();

  // Setup as totem pole structure
  i2c_smbus_write_byte_data(this->file_, MODE2, MODE2_OUTDRV_VAL);
  std::this_thread::sleep_for(std::chrono::milliseconds(25));

  i2c_smbus_write_byte_data(this->file_, MODE1, ~MODE1_SLEEP_VAL);
  std::this_thread::sleep_for(std::chrono::milliseconds(25));

  // Set all channels to 0
  i2c_smbus_write_byte_data(this->file_, CHANNEL_ALL, 0);
  i2c_smbus_write_byte_data(this->file_, CHANNEL_ALL + 1, 0);
  i2c_smbus_write_byte_data(this->file_, CHANNEL_ALL + 2, 0);
  i2c_smbus_write_byte_data(this->file_, CHANNEL_ALL + 3, 0);
}

void PCA9685Interface::stop() {
  // Set all channels to 0
  i2c_smbus_write_byte_data(this->file_, CHANNEL_ALL, 0);
  i2c_smbus_write_byte_data(this->file_, CHANNEL_ALL + 1, 0);
  i2c_smbus_write_byte_data(this->file_, CHANNEL_ALL + 2, 0);
  i2c_smbus_write_byte_data(this->file_, CHANNEL_ALL + 3, 0);
}

void PCA9685Interface::setOscillatorFrequency(float frequency,
                                              bool externalClock) {
  this->oscillatorFrequency_ = frequency;
  this->externalClock_ = externalClock;
}

void PCA9685Interface::restart() {
  __s32 data = i2c_smbus_read_byte_data(this->file_, MODE1);
  if ((data & MODE1_RESTART_VAL) >> MODE1_RESTART) {
    this->sleep();
  }
  // set RESTART bit of MODE1 up to complete restart.
  i2c_smbus_write_byte_data(this->file_, MODE1, data | MODE1_RESTART_VAL);
}

void PCA9685Interface::sleep() {
  // read current MODE1
  __s32 sleepOrAwake = i2c_smbus_read_byte_data(this->file_, MODE1);
  // set SLEEP bit of the current model to 1
  i2c_smbus_write_byte_data(this->file_, MODE1, sleepOrAwake | MODE1_SLEEP_VAL);
  // Sleep to wait for the oscillator to stabilize
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

void PCA9685Interface::wake() {
  // read current MODE1
  __s32 sleepOrAwake = i2c_smbus_read_byte_data(this->file_, MODE1);
  // set SLEEP bit of the current model to 0
  i2c_smbus_write_byte_data(this->file_, MODE1,
                            sleepOrAwake & ~MODE1_SLEEP_VAL);
  // Sleep to wait for the oscillator to stabilize
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

void PCA9685Interface::setInternalClockFrequency(uint8_t prescale) {
  __s32 data = i2c_smbus_read_byte_data(this->file_, MODE1);

  // Set sleep without restart
  __s32 dataSleepNoRestart = (data & ~MODE1_RESTART_VAL) | MODE1_SLEEP_VAL;
  i2c_smbus_write_byte_data(this->file_, MODE1, dataSleepNoRestart);

  // set prescale
  i2c_smbus_write_byte_data(this->file_, PRE_SCALE, prescale);

  // write data back
  i2c_smbus_write_byte_data(this->file_, MODE1, data);
  // wait for osciallator to stabilize
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  // Restart and set auto increment up
  i2c_smbus_write_byte_data(this->file_, MODE1,
                            data | MODE1_RESTART | MODE1_AI);
}

void PCA9685Interface::setExternalClockFrequency(uint8_t prescale) {
  __s32 data = i2c_smbus_read_byte_data(this->file_, MODE1);

  // Set sleep without restart
  __s32 dataSleepNoRestart = (data & ~MODE1_RESTART_VAL) | MODE1_SLEEP_VAL;
  i2c_smbus_write_byte_data(this->file_, MODE1, dataSleepNoRestart);

  // Set extclk bit
  dataSleepNoRestart = dataSleepNoRestart | MODE1_EXTCLK_VAL;
  i2c_smbus_write_byte_data(this->file_, MODE1, dataSleepNoRestart);

  // Set prescale
  i2c_smbus_write_byte_data(this->file_, PRE_SCALE, prescale);
  // stabilize oscillator
  std::this_thread::sleep_for(std::chrono::milliseconds(1));

  // Unsleep, restart and autoincrement
  i2c_smbus_write_byte_data(
      this->file_, MODE1,
      (dataSleepNoRestart & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI);
}

void PCA9685Interface::setChannelValue(uint8_t channel, uint8_t* values) {
  i2c_smbus_write_i2c_block_data(this->file_, CHANNEL_0 + (channel * 4), 4,
                                 values);
}

void PCA9685Interface::setPWMFrequency(float pwmFrequency) {
  this->pwmFrequency_ = pwmFrequency;
}

void PCA9685Interface::setFrequency() {
  float frequency = std::min(std::max(1.0f, this->pwmFrequency_), 3500.0f);
  float prescaleValue =
      ((this->oscillatorFrequency_ / (frequency * 4096.0)) + 0.5) - 1;
  prescaleValue = std::min(std::max(3.0f, prescaleValue), 255.0f);
  uint8_t prescale = static_cast<uint8_t>(prescaleValue);

  if (this->externalClock_) {
    this->setExternalClockFrequency(prescale);
  } else {
    this->setInternalClockFrequency(prescale);
  }
}

PCA9685Channel* PCA9685Interface::createChannel(
    const PCA9685Channel::Configuration& channelBuilder) {
  return new PCA9685Channel(
      channelBuilder,
      std::bind(&PCA9685Interface::setChannelValue, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&PCA9685Interface::setPWMFrequency, this,
                std::placeholders::_1));
}

}  // namespace communication
}  // namespace motor_controllers
