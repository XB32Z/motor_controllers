#include <motor_controllers/communication/pigpio_pwm_channel.h>
#include <pigpio.h>

#include <array>          // std::array
#include <cmath>          // std::floor
#include <unordered_map>  // std::unordered_map

static const std::unordered_map<uint8_t, std::array<uint32_t, 18>>
    freqPerSampleRate = {{1,
                          {50, 100, 200, 250, 400, 500, 800, 1000, 1250, 1600,
                           2000, 2500, 4000, 5000, 8000, 10000, 20000, 40000}},
                         {2,
                          {25, 50, 100, 125, 200, 250, 400, 500, 625, 800, 1000,
                           1250, 2000, 2500, 4000, 5000, 10000, 20000}},

                         {4,
                          {13, 25, 50, 63, 100, 125, 200, 250, 313, 400, 500,
                           625, 1000, 1250, 2000, 2500, 5000, 10000}},

                         {5,
                          {10, 20, 40, 50, 80, 100, 160, 200, 250, 320, 400,
                           500, 800, 1000, 1600, 2000, 4000, 8000}},

                         {8,
                          {6, 13, 25, 31, 50, 63, 100, 125, 156, 200, 250, 313,
                           500, 625, 1000, 1250, 2500, 5000}},

                         {10,
                          {5, 10, 20, 25, 40, 50, 80, 100, 125, 160, 200, 250,
                           400, 500, 800, 1000, 2000, 4000}}};

namespace motor_controllers {

namespace communication {

PiGPIOPWMChannel::PiGPIOPWMChannel(const Configuration& builder,
                                   uint8_t sampleRate)
    : IPWMSignalChannel(),
      pinNumber_(builder.pinNumber),
      range_(builder.range),
      altMode_(builder.altMode),
      frequency_(500),
      isHardware_(builder.isHardware),
      sampleRate_(sampleRate) {}

PiGPIOPWMChannel::~PiGPIOPWMChannel() {
  if (!this->isCommunicationClosed()) {
    gpioPWM(this->pinNumber_, 0);
  }
}

void PiGPIOPWMChannel::setPWMFrequency(float frequency) {
  const uint32_t f = static_cast<unsigned int>(std::floor(frequency));

  const auto& allowedFreq = freqPerSampleRate.find(this->sampleRate_);
  if (allowedFreq != freqPerSampleRate.end()) {
    const auto& lower = std::lower_bound(allowedFreq->second.begin(),
                                         allowedFreq->second.end(), f);
    if (lower != allowedFreq->second.end()) {
      this->frequency_ = *lower;
    } else {
      throw std::runtime_error("Frequency too high");
    }
  } else {
    throw std::runtime_error("Sample rate not supported");
  }

  if (!this->isHardware_) {
    gpioSetPWMfrequency(this->pinNumber_, this->frequency_);
  }
}

void PiGPIOPWMChannel::setPWM(float start, float end) {
  const unsigned int dc = static_cast<unsigned int>(std::floor(end - start));
  if (this->isHardware_) {
    gpioHardwarePWM(this->pinNumber_, this->frequency_, dc);
  } else {
    gpioPWM(this->pinNumber_, dc);
  }
}

void PiGPIOPWMChannel::setDutyCyle(float dutyCycle) {
  if (this->isCommunicationClosed()) {
    throw std::runtime_error(
        "PiGPIOPWMChannel: communication is closed, cannot set duty cycle");
  }

  dutyCycle = std::max(std::min(dutyCycle, 1.0f), 0.0f);
  const unsigned int dc =
      static_cast<unsigned int>(std::floor(dutyCycle * this->range_));
  if (this->isHardware_) {
    gpioHardwarePWM(this->pinNumber_, this->frequency_, dc);
  } else {
    gpioPWM(this->pinNumber_, dc);
  }
}

float PiGPIOPWMChannel::getMinValue() const { return 0; }

float PiGPIOPWMChannel::getMaxValue() const { return this->range_; }

void PiGPIOPWMChannel::initialize() {
  gpioSetMode(this->pinNumber_, static_cast<unsigned int>(this->altMode_));
  if (this->isHardware_) {
    gpioHardwarePWM(this->pinNumber_, this->frequency_, 0);
  } else {
    gpioPWM(this->pinNumber_, 0);
    gpioSetPWMrange(this->pinNumber_, this->range_);
  }
}

}  // namespace communication
}  // namespace motor_controllers
