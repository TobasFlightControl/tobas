// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "./rosbag_recorder.hpp"

namespace tobas
{
RosbagRecorderNode::RosbagRecorderNode(const rclcpp::NodeOptions& options)
  : super("rosbag_recorder", nodeOptions_Default(options))
  , ns_(std::string(get_namespace()) + "/")
  , rosbag_dir_(linux::isSuperUser() ? kRosbagDirRoot : ros2::expandUser(kRosbagDirHome))
{
  // Register publishers.
  rosbag_state_pub_ = createPublisher<tobas_msgs::msg::RosbagState>(topic::kRosbagState);

  // Resister subscribers.
  // Establish topic connections before recording starts because even local topic communication can introduce latency.
  // Template-heavy methods are split for separate compilation to reduce build-time memory usage.
  registerStateSubscribers();
  registerSensorSubscribers();
  registerStateSubscribers();
  registerCommandSubscribers();
  registerDebugSubscribers();

  // Register services.
  start_srv_ = createService<StartSrv>(service::kRosbagRecordStart, &self::startCb, this);
  stop_srv_ = createService<StopSrv>(service::kRosbagRecordStop, &self::stopCb, this);
  clean_srv_ = createService<CleanSrv>(service::kRosbagClean, &self::cleanCb, this);

  // Start main timer.
  main_timer_ = createTimer(kMainTimerPeriod, &self::mainTimerCb, this);
}

size_t RosbagRecorderNode::getDiskAvailableSize() const noexcept
{
  try {
    const auto info = fs::space(rosbag_dir_);
    return info.available;
  }
  catch (...) {
    TOBAS_ERROR("Failed to get the space information: ", rosbag_dir_);
    return 0;
  }
}

void RosbagRecorderNode::publishRosbagState()
{
  const auto cur_time = now();

  auto rosbag_state = std::make_unique<tobas_msgs::msg::RosbagState>();
  rosbag_state->header.stamp = cur_time;
  rosbag_state->recording = recording_;

  if (recording_) {
    const auto file_size = path::computeDirectorySize(file_path_);
    const auto available_size = getDiskAvailableSize();

    rosbag_state->file_path = file_path_;
    rosbag_state->duration = cur_time - start_time_;
    rosbag_state->file_size = file_size;
    rosbag_state->available_size = available_size;
    rosbag_state->message_count = msg_cnt_;

    if (available_size < kMinAvailableSize) {
      recording_ = false;
      TOBAS_WARN(
        "The recording was stopped because the available disk space dropped below ",
        kMinAvailableSize / 1'000'000,
        " MB.");

      try {
        writer_.close();
      }
      catch (const std::exception& e) {
        TOBAS_ERROR("Failed to close rosbag file: ", e.what());
      }
    }
  }

  rosbag_state_pub_->publish(std::move(rosbag_state));
}

void RosbagRecorderNode::startCb(const StartSrv::Request::ConstSharedPtr& req, const StartSrv::Response::SharedPtr& res)
{
  if (recording_) {
    res->success = false;
    res->message = "Rosbag recording is in progress.";
    return;
  }

  if (req->name.empty()) {
    res->success = false;
    res->message = "Please specify rosbag name.";
    return;
  }

  // Create the rosbag directory if it does not exist.
  if (!fs::exists(rosbag_dir_)) {
    std::error_code ec;
    if (!fs::create_directories(rosbag_dir_, ec)) {
      res->success = false;
      res->message = "Failed to create " + rosbag_dir_.string() + ": " + ec.message();
      return;
    }
  }

  // Check that a log with the same name does not exist.
  file_path_ = rosbag_dir_ / req->name;
  if (fs::exists(file_path_)) {
    if (req->overwrite) {
      fs::remove_all(file_path_);
    }
    else {
      res->success = false;
      res->message = file_path_.string() + " already exists.";
      return;
    }
  }

  // Check that the storage has enough free space.
  const auto available_size = getDiskAvailableSize();
  if (available_size < kMinAvailableSize) {
    res->success = false;
    res->message = "Recording cannot be started because there is not enough available disk space.";
    return;
  }

  rosbag2_storage::StorageOptions options;
  options.uri = file_path_;
  options.max_bagfile_size = req->max_file_size;
  options.max_cache_size = req->max_cache_size;

  try {
    writer_.open(options);
  }
  catch (const std::exception& e) {  // Storage capacity exceeded, etc.
    res->success = false;
    res->message = "Failed to open " + options.uri + ": " + e.what();
    return;
  }

  recording_ = true;
  start_time_ = now();
  msg_cnt_ = 0;

  TOBAS_INFO("Rosbag recording has started.");

  res->success = true;
  res->message.clear();

  publishRosbagState();
}

void RosbagRecorderNode::stopCb(const StopSrv::Request::ConstSharedPtr&, const StopSrv::Response::SharedPtr& res)
{
  if (!recording_) {
    res->success = false;
    res->message = "Rosbag recording is not in progress.";
    return;
  }

  try {
    writer_.close();
  }
  catch (const std::exception& e) {
    res->success = false;
    res->message = "Failed to close rosbag file: " + std::string(e.what());
    return;
  }

  recording_ = false;

  TOBAS_INFO("Rosbag recording has stopped.");

  res->success = true;
  res->message.clear();
  res->path = file_path_.string();

  publishRosbagState();
}

void RosbagRecorderNode::cleanCb(const CleanSrv::Request::ConstSharedPtr&, const CleanSrv::Response::SharedPtr& res)
{
  const auto clear_dir_res = path::clearDirectory(rosbag_dir_);

  res->success = clear_dir_res.has_value();
  res->message = clear_dir_res.error();
}

void RosbagRecorderNode::mainTimerCb()
{
  publishRosbagState();
}
}  // namespace tobas
