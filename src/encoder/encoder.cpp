#include <motor_controllers/encoder/encoder.h>

#include <array>   // std::array
#include <bitset>  // std::bitset
#include <functional>  // std::bind
#include <chrono>  // std::chrono

namespace motor_controllers {
namespace encoder {

// https://cdn.sparkfun.com/datasheets/Robotics/How%20to%20use%20a%20quadrature%20encoder.pdf
static const std::array<Direction, 16> QEM = {
    Direction::STOP,    Direction::BACKWARD, Direction::FORWARD,
    Direction::INVALID, Direction::FORWARD,  Direction::STOP,
    Direction::INVALID, Direction::BACKWARD, Direction::BACKWARD,
    Direction::INVALID, Direction::STOP,     Direction::FORWARD,
    Direction::INVALID, Direction::FORWARD,  Direction::BACKWARD,
    Direction::STOP};

Encoder::Encoder(std::unique_ptr<communication::IBinarySignalChannel> channelA,
                 std::unique_ptr<communication::IBinarySignalChannel> channelB)
    : channelA(std::move(channelA)),
      channelB(std::move(channelB)),
      running_(false),
      speed_(0.0),
      direction_(Direction::STOP) {}

float Encoder::getSpeed() const {
  std::lock_guard<std::mutex> lock(this->mtx_);
  return this->speed_;
}

Direction Encoder::getDirection() const {
  std::lock_guard<std::mutex> lock(this->mtx_);
  return this->direction_;
}

void Encoder::start() {
  this->running_ = true;
  this->thread_ = std::thread(std::bind(&Encoder::measureSpeed, this));
}

void Encoder::stop() {
  this->running_ = false;
  this->thread_.join();
}

void Encoder::measureSpeed() {
  // https://www.embeddedrelated.com/showarticle/158.php
  typedef std::chrono::high_resolution_clock clock_;

  uint cpt = 0;
  std::chrono::time_point<clock_> lastUpdate = clock_::now();
  const std::chrono::microseconds dt(5000);
  const float dts = std::chrono::duration_cast<std::chrono::seconds>(dt).count();

  std::bitset<4> data;
  data.reset();

  std::future<communication::BinarySignal> eventA, eventB;

  while (this->running_) {

    eventA = this->channelA->asyncDetectEvent();
    eventB = this->channelB->asyncDetectEvent();

    data.set(0, static_cast<bool>(eventA.get()));
    data.set(1, static_cast<bool>(eventB.get()));

    this->direction_ = QEM[static_cast<size_t>(data.to_ulong())];

    data <<= 2;

    auto now = clock_::now();
    if(now - lastUpdate >= dt) {
      std::lock_guard<std::mutex> lock(this->mtx_);
      lastUpdate = now;
      this->speed_ = cpt / dts;
      cpt = 0;
    }
    ++cpt;
  }
}

}  // namespace encoder
}  // namespace motor_controllers