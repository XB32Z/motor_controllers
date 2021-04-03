#include <motor_controllers/encoder/encoder.h>

#include <array>       // std::array
#include <bitset>      // std::bitset
#include <chrono>      // std::chrono
#include <cmath>       // std::round
#include <functional>  // std::bind

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

Encoder::Encoder(communication::IBinarySignalChannel::Ref channelA,
                 communication::IBinarySignalChannel::Ref channelB,
                 unsigned int resolution)
    : running_(false),
      channelA_(std::move(channelA)),
      channelB_(std::move(channelB)),
      resolution_(resolution),
      count_(0),
      speed_(0.0),
      direction_(Direction::STOP) {}

Encoder::Encoder(communication::IBinarySignalChannel::Ref channel,
                 unsigned int resolution)
    : running_(false),
      channelA_(std::move(channel)),
      resolution_(resolution),
      count_(0),
      speed_(0.0),
      direction_(Direction::FORWARD) {}

Encoder::~Encoder() { this->stop(); }

float Encoder::getSpeed() const {
  std::lock_guard<std::mutex> lock(this->mtx_);
  return this->speed_;
}

Direction Encoder::getDirection() const {
  std::lock_guard<std::mutex> lock(this->mtx_);
  return this->direction_;
}

ulong Encoder::getCount() const {
  std::lock_guard<std::mutex> lock(this->mtx_);
  return this->count_;
}

void Encoder::start(float freq) {
  std::chrono::microseconds samplingPeriod(
      static_cast<unsigned int>(std::round(1.0 / freq * 10e6)));
  this->running_ = true;
  if (this->channelA_) {
    if (this->channelB_) {
      this->thread_ = std::thread(std::bind(
          &Encoder::estimateVelocityQuadratureEncoder, this, samplingPeriod));
    } else {
      this->thread_ = std::thread(
          std::bind(&Encoder::estimateVelocityEncoder, this, samplingPeriod));
    }
  }
}

void Encoder::stop() {
  if (this->running_) {
    this->running_ = false;
    this->thread_.join();
  }
  this->count_ = 0;
  this->speed_ = 0.0;
  this->direction_ = (this->channelB_) ? Direction::STOP : Direction::FORWARD;
}

void Encoder::estimateVelocityQuadratureEncoder(
    std::chrono::microseconds samplingPeriod) {
  typedef std::chrono::high_resolution_clock clock_;

  // Velocity estimation
  //
  // https://www.embeddedrelated.com/showarticle/158.php
  // More efficient to have a fixed dt.
  uint cpt = 0;
  std::chrono::time_point<clock_> lastUpdate = clock_::now();
  std::chrono::microseconds dt;  // time elapsed since last estimation
  const float r = this->resolution_ * 4.0 * 10e-6;

  // Quadrature Encoder Matrix
  //
  // Index of the current direction is: 4*((2*prevA+prevB))+(2*curA+curB)
  // At any edge detection t, the value of both channel can be read to give
  // either 0, 1, 2 or 3.
  // The Quadrature Encoder Matrix is a 2D 4x4 matrix that relates the current
  // and previous values to a direction.
  // Index [i,j] in that matrix is 4*i+j in a 1D array.

  // It is convinient to write the successive values of A and B in a bitset that
  // we shift at each event to cheaply get the index in the QEM.
  std::bitset<4> qemIndex;
  qemIndex.reset();

  auto eventA = this->channelA_->asyncDetectEvent();
  auto eventB = this->channelB_->asyncDetectEvent();

  bool lastA = false, lastB = false;

  while (this->running_) {
    if (eventA.wait_for(std::chrono::microseconds(0)) ==
        std::future_status::ready) {
      qemIndex <<= 2;  // push previous data
      // Wait for A to switch then start next detection immediatly
      lastA = static_cast<bool>(eventA.get());
      eventA = this->channelA_->asyncDetectEvent();
      qemIndex.set(0, lastB);
      qemIndex.set(1, lastA);

      ++cpt;
      ++this->count_;
    }

    if (eventB.wait_for(std::chrono::microseconds(0)) ==
        std::future_status::ready) {
      qemIndex <<= 2;  // push previous data
      // Wait for B to switch then start next detection immediately
      lastB = static_cast<bool>(eventB.get());
      eventB = this->channelB_->asyncDetectEvent();
      qemIndex.set(0, lastB);
      qemIndex.set(1, lastA);

      ++cpt;
      ++this->count_;
    }

    auto now = clock_::now();
    if ((dt = std::chrono::duration_cast<std::chrono::microseconds>(
             now - lastUpdate)) >= samplingPeriod) {
      std::lock_guard<std::mutex> lock(this->mtx_);
      this->direction_ = QEM[static_cast<size_t>(qemIndex.to_ulong())];
      lastUpdate = now;
      this->speed_ = cpt / (dt.count() * r);
      cpt = 0;
    }
  }

  // Wait for wrap up
  this->channelA_->interuptEventDetection();
  this->channelB_->interuptEventDetection();
  eventA.get();
  eventB.get();
}

void Encoder::asyncEstimateVelocityQuadratureEncoder(
    std::chrono::microseconds samplingPeriod) {

  typedef std::chrono::high_resolution_clock clock_;

  uint cpt = 0;
  std::chrono::time_point<clock_> lastUpdate = clock_::now();
  std::chrono::microseconds dt;  // time elapsed since last estimation
  const float r = this->resolution_ * 2.0 * 10e-6;

  std::bitset<4> qemIndex;
  qemIndex.reset();

  auto eventA = this->channelA_->asyncDetectEvent();
  auto eventB = this->channelB_->asyncDetectEvent();

  bool lastA = false, lastB = false;

  while (this->running_) {
    qemIndex <<= 2;
    lastA = static_cast<bool>(eventA.get());
    eventA = this->channelA_->asyncDetectEvent();
    qemIndex.set(0, lastB);
    qemIndex.set(1, lastA);

    qemIndex <<= 2;
    lastB = static_cast<bool>(eventB.get());
    eventB = this->channelB_->asyncDetectEvent();
    qemIndex.set(0, lastB);
    qemIndex.set(1, lastA);

    auto now = clock_::now();
    if ((dt = std::chrono::duration_cast<std::chrono::microseconds>(
             now - lastUpdate)) >= samplingPeriod) {
      std::lock_guard<std::mutex> lock(this->mtx_);
      this->direction_ = QEM[static_cast<size_t>(qemIndex.to_ulong())];
      lastUpdate = now;
      this->speed_ = cpt / (dt.count() * r);
      cpt = 0;
    }
    ++cpt;
    ++this->count_;
  }

  // Wait for wrap up
  this->channelA_->interuptEventDetection();
  this->channelB_->interuptEventDetection();
  eventA.get();
  eventB.get();
}

void Encoder::estimateVelocityEncoder(
    std::chrono::microseconds samplingPeriod) {
  typedef std::chrono::high_resolution_clock clock_;

  // Velocity estimation
  //
  // https://www.embeddedrelated.com/showarticle/158.php
  // More efficient to have a fixed dt.
  uint cpt = 0;
  std::chrono::time_point<clock_> lastUpdate = clock_::now();
  std::chrono::microseconds dt;  // time elapsed since last estimation
  const float r = this->resolution_ * 2.0 * 10e-6;

  while (this->running_) {
    this->channelA_->asyncDetectEvent().get();

    auto now = clock_::now();
    if ((dt = std::chrono::duration_cast<std::chrono::microseconds>(
             now - lastUpdate)) >= samplingPeriod) {
      std::lock_guard<std::mutex> lock(this->mtx_);
      lastUpdate = now;
      this->speed_ = cpt / (dt.count() * r);
      cpt = 0;
    }
    ++cpt;
    ++this->count_;
  }
}

}  // namespace encoder
}  // namespace motor_controllers