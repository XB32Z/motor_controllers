/**
 * @file channel_builder.h
 * @author Pierre Venet
 * @brief
 * @version 0.1
 * @date 2021-03-01
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <motor_controllers/communication/i_communication_interface.h>

#include <algorithm>
#include <functional>
#include <vector>

namespace motor_controllers {
namespace communication {

/**
 * @brief Builder interface for channels
 *
 * Because we cannot return unique_ptr in a covariant class hierarchy, we use
 * this template workaround.
 *
 * This class wrapps the logic necessary to create CommunicationChannel
 * unique_ptr and manage their life cycles. The interface offers only unique_ptr
 * in order for the users to not inadvertently copy channels and try to
 * concurrently write to them.
 *
 * This class creates a pointer to a new channel and stores it internally. The
 * method configureChannel wrap that pointer in a unique_ptr and set the
 * destructor to a custom method, such that, when the channel is not referenced
 * anymore this class can properly deregister it.
 *
 * Vice versa, if the CommunicationInterface is destroyed before its channels,
 * they are all "informed", which prevents them from attempting to write data
 * and prevents double freeing them.
 *
 * @tparam ChannelMode The type of channel that this class will produce
 * @tparam ChannelBuilder The builder object used to create a ChannelMode
 */
template <typename ChannelMode, typename Builder>
class ChannelBuilder : public ICommunicationInterface {
 public:
  virtual ~ChannelBuilder() {
    // Close all the channels which are not yet deregistered.
    for (auto& channel : this->channels_) {
      channel->closeCommunication();
    }
  }

 public:
  std::unique_ptr<ChannelMode, std::function<void(ChannelMode*)>>
  configureChannel(const Builder& channelBuilder) {
    // Call the method of the implementation of this class to get the properly
    // configured ChannelMode.
    ChannelMode* channel = this->createChannel(channelBuilder);

    this->channels_.emplace_back(channel);

    // Create the unique_ptr object. Upon destruction of channel, the lambda
    // expression is called.
    std::unique_ptr<ChannelMode, std::function<void(ChannelMode*)>> res(
        channel, [this](ChannelMode* ptr) {
          if (!ptr->isCommunicationClosed()) {
            // if communication is closed, this object is already destroyed
            this->unregisterChannel(ptr);
          }
          delete ptr;
        });

    return res;
  }

 private:
  /**
   * @brief Create a ChannelMode object
   *
   * Implement this method for the pattern to work.
   *
   * @param channel
   * @return ChannelMode*
   */
  virtual ChannelMode* createChannel(const Builder& channel) = 0;

  /**
   * @brief Unregister a channel from the communication
   *
   * @param channel
   */
  virtual void unregisterChannel(ChannelMode* channel) {
    // This method is called when a unique_ptr managing a channel is destroyed.
    auto result =
        std::find_if(this->channels_.begin(), this->channels_.end(),
                     [channel](const auto& entry) { return entry == channel; });

    if (result != this->channels_.end()) {
      channel->closeCommunication();
      this->channels_.erase(result);
    } else {
      throw std::runtime_error(
          "FATAL: channel is deleting but was not created.");
    }
  }

 protected:
  std::vector<ChannelMode*> channels_;
};

}  // namespace communication

}  // namespace motor_controllers