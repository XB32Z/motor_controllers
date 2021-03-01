/**
 * @file communication_channel_builder.h
 * @author Pierre Venet
 * @brief
 * @version 0.1
 * @date 2021-03-01
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <motor_controllers/communication/communication_interface.h>

#include <algorithm>
#include <functional>
#include <map>

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
 * @tparam ChannelType
 */
template <typename ChannelType>
class CommunicationChannelBuilder : public CommunicationInterface {
 public:
  virtual ~CommunicationChannelBuilder() {
    // Close all the channels which are not yet deregistered.
    for (auto& channel : this->channels_) {
      channel.second->closeCommunication();
    }
  }

 public:
  std::unique_ptr<ChannelType, std::function<void(ChannelType*)>>
  configureChannel(uint8_t channelNumber) {
    if (this->channels_.find(channelNumber) != this->channels_.end())
      throw std::runtime_error("Channel already configured");

    // Call the method of the implementation of this class to get the properly
    // configured ChannelType.
    ChannelType* channel = this->createChannel(channelNumber);

    this->channels_.emplace(channelNumber, channel);

    // Create the unique_ptr object. Upon destruction of channel, the lambda
    // expression is called.
    std::unique_ptr<ChannelType, std::function<void(ChannelType*)>> res(
        channel, [this](ChannelType* ptr) {
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
   * @brief Create a ChannelType object
   * 
   * Implement this method for the pattern to work.
   *
   * @param channel
   * @return ChannelType*
   */
  virtual ChannelType* createChannel(uint8_t channel) = 0;

  /**
   * @brief Unregister a channel from the communication
   * 
   * @param channel 
   */
  virtual void unregisterChannel(ChannelType* channel) {
    // This method is called when a unique_ptr managing a channel is destroyed.
    auto result = std::find_if(this->channels_.begin(), this->channels_.end(),
                               [channel](const auto& mapChannel) {
                                 return mapChannel.second == channel;
                               });

    if (result != this->channels_.end()) {
      channel->closeCommunication();
      this->channels_.erase(result);
    } else {
      throw std::runtime_error(
          "FATAL: channel is deleting but was not created.");
    }
  }

 protected:
  std::map<uint8_t, ChannelType*> channels_;
};
}  // namespace communication

}  // namespace motor_controllers