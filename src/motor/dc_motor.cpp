#include <motor_controllers/motor/dc_motor.h>

#include <chrono>      // std::chrono
#include <functional>  // std::bind
#include <memory>      // std::move

namespace motor_controllers {
namespace motor {
DCMotor::DCMotor(Configuration& conf)
    : pwmChannel_(std::move(conf.pwmChannel)),
      encoder_(std::move(conf.encoder)),
      directionControl_(std::move(conf.directionControl)),
      forwardConfiguration_(conf.forwardConfiguration),
      backwardConfiguration_(conf.backwardConfiguration),
      stopConfiguration_(conf.stopConfiguration),
      isRunning_(false),
      minDutyCycle_(conf.minDutyCycle),
      coefSpeedToDutyCycle_((1.0 - conf.minDutyCycle) / conf.maxSpeed),
      maxSpeed_(conf.maxSpeed),
      targetSpeed_(0.0),
      previousError_(0.0),
      integral_(0.0),
      Kp_(conf.Kp),
      Ki_(conf.Ki),
      Kd_(conf.Kd) {}

DCMotor::~DCMotor() {}

void DCMotor::start() {
  this->isRunning_ = true;
  this->controlThread_ = std::thread(std::bind(&DCMotor::controlLoop, this));
}

void DCMotor::stop() {
  if (this->isRunning_) {
    this->isRunning_ = false;
    this->controlThread_.join();
  }
}

void DCMotor::setSpeed(double speed) {
  std::lock_guard<std::mutex> lock(this->mtx_);
  this->targetSpeed_ = 0.0;
}

double DCMotor::getSpeed() const {
  switch (this->encoder_->getDirection()) {
    case encoder::Direction::BACKWARD:
      return -this->encoder_->getSpeed();
    case encoder::Direction::INVALID:
      // what to do!?
    default:
      return this->encoder_->getSpeed();
  }
}

void DCMotor::controlLoop(std::chrono::microseconds dt) {
  typedef std::chrono::high_resolution_clock clock_;
  std::chrono::time_point<clock_> lastUpdate = clock_::now();

  const double ratio = dt.count() * 1e-6;

  this->previousError_ = 0.0;
  this->integral_ = 0.0;

  while (this->isRunning_) {
    const double currentSpeed = this->getSpeed();

    std::lock_guard<std::mutex> lock(this->mtx_);

    const auto error = this->targetSpeed_ - currentSpeed;

    this->integral_ += error * ratio;

    const auto speed = this->Kp_ * error + this->Ki_ * this->integral_ +
                       this->Kd_ * (error - this->previousError_) / ratio;

    this->previousError_ = error;

    const double dutyCycle =
        speed * this->coefSpeedToDutyCycle_ + this->minDutyCycle_;

    if (dutyCycle < 0 && currentSpeed > 0) {
      this->backward();
    } else if (dutyCycle > 0 && currentSpeed < 0) {
      this->forward();
    }

    this->pwmChannel_->setDutyCyle(std::abs(dutyCycle));

    std::this_thread::sleep_until(lastUpdate + dt);
    lastUpdate = clock_::now();
  }
}

void DCMotor::forward() {
  for (size_t i = 0; i < this->directionControl_.size(); ++i) {
    this->directionControl_[i]->set(this->forwardConfiguration_[i]);
  }
}

void DCMotor::backward() {
  for (size_t i = 0; i < this->directionControl_.size(); ++i) {
    this->directionControl_[i]->set(this->backwardConfiguration_[i]);
  }
}

void DCMotor::stop() {
  for (size_t i = 0; i < this->directionControl_.size(); ++i) {
    this->directionControl_[i]->set(this->stopConfiguration_[i]);
  }
}

}  // namespace motor
}  // namespace motor_controllers