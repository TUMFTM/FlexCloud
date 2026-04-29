/*
 * TUM Autonomous Motorsport Georeferencing Tool
 * Copyright (C) 2026 Maximilian Leitenstern
 *
 * Adapted from PointCloudCrafter (Apache 2.0):
 *   https://github.com/TUMFTM/PointCloudCrafter — utils/rosbag_reader.hpp
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#pragma once

#include <functional>
#include <memory>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/utilities.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace flexcloud::tools
{
/**
 * @brief Pair of a deserialized ROS message and its containing rosbag entry.
 */
template <typename T>
struct RosbagReaderMsg
{
  const rosbag2_storage::SerializedBagMessage & bag_msg;
  const T & ros_msg;
};

/**
 * @brief Type-erased base for per-topic message handlers.
 */
class SerializedMessageHandlerBase
{
public:
  virtual ~SerializedMessageHandlerBase() = default;
  virtual void handle_serialized_message(
    const rosbag2_storage::SerializedBagMessage & bag_msg,
    const rclcpp::SerializedMessage & ser_msg) = 0;
};

/**
 * @brief Deserializes incoming messages of type T and forwards them to a callback.
 */
template <typename T>
class SerializedMessageHandler : public SerializedMessageHandlerBase
{
public:
  explicit SerializedMessageHandler(std::function<void(const RosbagReaderMsg<T> &)> callback)
  : callback_(std::move(callback))
  {
  }

  void handle_serialized_message(
    const rosbag2_storage::SerializedBagMessage & bag_msg,
    const rclcpp::SerializedMessage & ser_msg) override
  {
    T msg;
    serialization_.deserialize_message(&ser_msg, &msg);
    callback_({bag_msg, msg});
  }

private:
  rclcpp::Serialization<T> serialization_;
  std::function<void(const RosbagReaderMsg<T> &)> callback_;
};

/**
 * @brief Single-pass callback-based reader for ROS 2 bags.
 *
 * Register one or more `add_listener<T>(topic, cb)` callbacks, then call
 * `process()` to iterate through the bag once and dispatch each message
 * (filtered to the registered topics) to the matching handlers.
 */
class RosbagReader
{
public:
  explicit RosbagReader(const std::string & bag_path) { reader_.open(bag_path); }

  template <typename T>
  void add_listener(
    const std::string & topic_name, std::function<void(const RosbagReaderMsg<T> &)> callback)
  {
    handlers_[topic_name].push_back(
      std::make_shared<SerializedMessageHandler<T>>(std::move(callback)));
  }

  /// Stop processing early (e.g. once enough messages have been collected).
  void set_state(bool state) { running_ = state; }

  /// Bag metadata of the opened reader (topic types, durations, ...).
  const rosbag2_storage::BagMetadata & get_metadata() { return reader_.get_metadata(); }

  /// Read through the bag, dispatching messages on registered topics to their handlers.
  void process()
  {
    int64_t first_msg_time = -1;
    rclcpp::Clock wall_clock{};

    std::vector<std::string> topics;
    topics.reserve(handlers_.size());
    for (auto & entry : handlers_) {
      topics.push_back(entry.first);
    }
    reader_.set_filter({topics});

    const double total_duration =
      static_cast<double>(reader_.get_metadata().duration.count() / 1.0e9);
    running_ = true;

    while (reader_.has_next() && rclcpp::ok() && running_) {
      auto bag_msg = reader_.read_next();
      rclcpp::SerializedMessage ser_msg{*bag_msg->serialized_data};

      if (first_msg_time < 0) {
        first_msg_time = bag_msg->send_timestamp;
      }
      const double time_done =
        static_cast<double>((bag_msg->send_timestamp - first_msg_time) / 1.0e9);
      RCLCPP_INFO_THROTTLE(
        logger_, wall_clock, 1000, "Processed %.3f sec of bag [% 5.1f%%]", time_done,
        100 * time_done / total_duration);

      auto it = handlers_.find(bag_msg->topic_name);
      if (it == handlers_.end()) {
        continue;
      }
      for (auto & handler : it->second) {
        handler->handle_serialized_message(*bag_msg, ser_msg);
      }
    }
  }

private:
  std::unordered_map<std::string, std::vector<std::shared_ptr<SerializedMessageHandlerBase>>>
    handlers_;
  rclcpp::Logger logger_ = rclcpp::get_logger("flexcloud_rosbag_reader");
  rosbag2_cpp::Reader reader_;
  bool running_{true};
};
}  // namespace flexcloud::tools
