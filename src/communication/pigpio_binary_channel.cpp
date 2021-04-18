#include <motor_controllers/communication/pigpio_binary_channel.h>
#include <pigpio.h>

#include <condition_variable>  // std::condition_variable
#include <mutex>               // std::mutex, std::unique_lock
#include <stdexcept>

namespace motor_controllers {

namespace communication {

PiGPIOBinaryChannel::PiGPIOBinaryChannel(const Configuration& builder)
    : IBinarySignalChannel(builder.channelMode),
      pinNumber_(builder.pinNumber),
      eventDetectValue_(builder.eventDetectValue),
      detectEventThreadAlive_(false),
      detectEventAsyncAlive_(false) {}

PiGPIOBinaryChannel::~PiGPIOBinaryChannel() {
  if (!this->isCommunicationClosed()) {
    if (this->getChannelMode() == ChannelMode::OUTPUT) {
      this->setInternal(BinarySignal::BINARY_LOW);
    }
    this->interuptEventDetection();
  }
}

void PiGPIOBinaryChannel::set(const BinarySignal& value) {
  if (this->isCommunicationClosed()) {
    throw std::runtime_error(
        "PiGPIOBinaryChannel: communication is closed, cannot set value");
  }
  if (this->getChannelMode() == ChannelMode::OUTPUT) {
    this->setInternal(value);
  } else {
    throw std::runtime_error("Cannot write on a INPUT channel");
  }
}

BinarySignal PiGPIOBinaryChannel::get() {
  if (this->isCommunicationClosed()) {
    throw std::runtime_error(
        "PiGPIOBinaryChannel: communication is closed, cannot get value");
  }

  // Can read whether pin is input or output
  return static_cast<BinarySignal>(gpioRead(this->pinNumber_));
}

std::future<BinarySignal> PiGPIOBinaryChannel::asyncDetectEvent() {
  this->detectEventAsyncAlive_ = true;

  return std::async([this]() {
    std::unique_lock<std::mutex> lck(this->eventDetectionMutex_);
    this->eventDetectionCondVar_.wait(
        lck, [this] { return this->eventDetectionReady_; });
    this->eventDetectionReady_ = false;
    return this->eventDetectCurrentValue_;
  });
}

void PiGPIOBinaryChannel::onDetectEvent(
    const std::function<void(BinarySignal)>& callback) {
  if (this->isCommunicationClosed()) {
    throw std::runtime_error(
        "PiGPIOBinaryChannel: communication is closed, cannot detect events");
  }

  if (this->detectEventAsyncAlive_) {
    throw std::runtime_error("Cannot run thread and async simultaneously");
  }

  if (this->detectEventThreadAlive_) {  // thread already running
    this->detectEventThreadAlive_ = false;
    this->detectEventThread_.join();
  }

  this->detectEventThreadAlive_ = true;

  this->detectEventThread_ = std::thread([this, &callback]() {
    while (this->detectEventThreadAlive_) {
      std::unique_lock<std::mutex> lck(this->eventDetectionMutex_);
      this->eventDetectionCondVar_.wait(
          lck, [this] { return this->eventDetectionReady_; });
      callback(this->eventDetectCurrentValue_);
      this->eventDetectionReady_ = false;
    }
  });
}

void PiGPIOBinaryChannel::initialize() {
  if (this->getChannelMode() == ChannelMode::INPUT) {
    this->setupInput();
  } else if (this->getChannelMode() == ChannelMode::OUTPUT) {
    this->setupOutput();
  } else if (this->getChannelMode() == ChannelMode::EVENT_DETECT) {
    this->setupInput();
    this->setupEventDetection();
  }
}

void PiGPIOBinaryChannel::setInternal(const BinarySignal& value) {
  if (value == BinarySignal::BINARY_HIGH) {
    gpioWrite(this->pinNumber_, 1);
  } else {
    gpioWrite(this->pinNumber_, 0);
  }
}

void PiGPIOBinaryChannel::interuptEventDetection() {
  if (this->detectEventThreadAlive_) {
    this->detectEventThreadAlive_ = false;
    this->detectEventThread_.join();
  }

  if (this->detectEventAsyncAlive_) {
    this->detectEventAsyncAlive_ = false;
  }
}

void PiGPIOBinaryChannel::setupInput() {
  gpioSetMode(this->pinNumber_, PI_INPUT);
  gpioSetPullUpDown(this->pinNumber_, PI_PUD_UP);
}

void PiGPIOBinaryChannel::setupOutput() {
  gpioSetMode(this->pinNumber_, PI_OUTPUT);
}

void PiGPIOBinaryChannel::setupEventDetection() {
  gpioAlertFuncEx_t fct;

  switch (this->eventDetectValue_) {
    case EventDetectType::EVENT_HIGH:
      fct = [](int gpio, int level, uint32_t tick, void* userdata) {
        if (level == 1) {
          static_cast<PiGPIOBinaryChannel*>(userdata)->onGPIOChangeState(
              gpio, level, tick);
        }
      };
      break;
    case EventDetectType::EVENT_LOW:
      fct = [](int gpio, int level, uint32_t tick, void* userdata) {
        if (level == 0) {
          static_cast<PiGPIOBinaryChannel*>(userdata)->onGPIOChangeState(
              gpio, level, tick);
        }
      };
      break;
    case EventDetectType::EVENT_RISING_EDGE:
      fct = [](int gpio, int level, uint32_t tick, void* userdata) {
        if (level == 1) {
          static_cast<PiGPIOBinaryChannel*>(userdata)->onGPIOChangeState(
              gpio, level, tick);
        }
      };
      break;
    case EventDetectType::EVENT_FALING_EDGE:
      fct = [](int gpio, int level, uint32_t tick, void* userdata) {
        if (level == 0) {
          static_cast<PiGPIOBinaryChannel*>(userdata)->onGPIOChangeState(
              gpio, level, tick);
        }
      };
      break;
    case EventDetectType::EVENT_BOTH_EDGES:
      fct = [](int gpio, int level, uint32_t tick, void* userdata) {
        static_cast<PiGPIOBinaryChannel*>(userdata)->onGPIOChangeState(
            gpio, level, tick);
      };
      break;

    default:
      throw std::runtime_error("Not supported event detect type");
  }

  this->eventDetectCurrentValue_ = this->get();
  this->eventDetectionReady_ = false;

  gpioSetAlertFuncEx(this->pinNumber_, fct, this);
}

void PiGPIOBinaryChannel::onGPIOChangeState(int gpio, int level,
                                            uint32_t) {
  // pigpio calls this callback at the specified frequency.
  // This means that the state might have changed multiple times
  // between two callbacks.

  if (gpio != this->pinNumber_) {
    return;
  }

  std::unique_lock<std::mutex> lck(this->eventDetectionMutex_);

  this->eventDetectCurrentValue_ = static_cast<BinarySignal>(level);
  this->eventDetectionReady_ = true;
  this->eventDetectionCondVar_.notify_one();  // there should be only one
}

}  // namespace communication
}  // namespace motor_controllers
