#include <motor_controllers/communication/pigpio_pwm_channel.h>
#include <pigpio.h>

#include <array>          // std::array
#include <cmath>          // std::floor
#include <unordered_map>  // std::unordered_map

static const std::unordered_map<uint8_t, std::array<uint32_t, 18>>
    freqPerSampleRate = {{1,
                          {40000, 20000, 10000, 8000, 5000, 4000, 2500, 2000,
                           1600, 1250, 1000, 800, 500, 400, 250, 200, 100, 50}},
                         {2,
                          {20000, 10000, 5000, 4000, 2500, 2000, 1250, 1000,
                           800, 625, 500, 400, 250, 200, 125, 100, 50, 25}},

                         {4,
                          {10000, 5000, 2500, 2000, 1250, 1000, 625, 500, 400,
                           313, 250, 200, 125, 100, 63, 50, 25, 13}},

                         {5,
                          {8000, 4000, 2000, 1600, 1000, 800, 500, 400, 320,
                           250, 200, 160, 100, 80, 50, 40, 20, 10}},

                         {8,
                          {5000, 2500, 1250, 1000, 625, 500, 313, 250, 200, 156,
                           125, 100, 63, 50, 31, 25, 13, 6}},

                         {10,
                          {4000, 2000, 1000, 800, 500, 400, 250, 200, 160, 125,
                           100, 80, 50, 40, 25, 20, 10, 5}}};

namespace motor_controllers {

namespace communication {

PiGPIOPWMChannel::PiGPIOPWMChannel(const Configuration& builder,
                                   uint8_t sampleRate)
    : IPWMSignalChannel(),
      pinNumber_(builder.pinNumber),
      range_(builder.range),
      altMode_(builder.altMode_),
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