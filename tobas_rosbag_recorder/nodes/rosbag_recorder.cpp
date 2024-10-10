#include <rosbag2_cpp/writer.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tobas_path_tools/core.hpp>
#include <tobas_ros2_tools/filesystem.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>

#include <tobas_std_msgs/msg/message.hpp>
#include <tobas_msgs/msg/imu.hpp>

#include <tobas_msgs/srv/bag_record_start.hpp>
#include <tobas_msgs/srv/bag_record_stop.hpp>

namespace fs = std::filesystem;

class ROSBagRecorderNode : public tobas::BaseNode
{
  using self = ROSBagRecorderNode;
  using super = tobas::BaseNode;

  static constexpr size_t kMaxDirSize = 5'000'000'000;  // [byte]

public:
  explicit ROSBagRecorderNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  using StartSrv = tobas_msgs::srv::BagRecordStart;
  using StopSrv = tobas_msgs::srv::BagRecordStop;
  using CleanSrv = std_srvs::srv::Trigger;

private:
  const std::string ns_;
  const fs::path rosbag_dir_;

  rosbag2_cpp::Writer writer_;
  bool is_recording_ = false;

  ros2::SubscriberPtr<tobas_msgs::msg::Imu> imu_sub_;

  ros2::ServiceServerPtr<StartSrv> start_srv_;
  ros2::ServiceServerPtr<StopSrv> stop_srv_;
  ros2::ServiceServerPtr<CleanSrv> clean_srv_;

  void imuCb(const tobas_msgs::msg::Imu::ConstSharedPtr& msg);

  void startCb(const StartSrv::Request::ConstSharedPtr& req, const StartSrv::Response::SharedPtr& res);
  void stopCb(const StopSrv::Request::ConstSharedPtr& req, const StopSrv::Response::SharedPtr& res);
  void cleanCb(const CleanSrv::Request::ConstSharedPtr& req, const CleanSrv::Response::SharedPtr& res);
};

ROSBagRecorderNode::ROSBagRecorderNode(const rclcpp::NodeOptions& options)
  : super("rosbag_recorder", options),
    ns_(std::string(get_namespace()) + "/"),
    rosbag_dir_(ros2::expandUser(tobas::kROSBagDir))
{
  // Register subscriptions
  imu_sub_ = createSubscriber(tobas::kImuTopic, &self::imuCb, this);

  // Register services
  start_srv_ = createService<StartSrv>(tobas::kROSBagRecordStartSrv, &self::startCb, this);
  stop_srv_ = createService<StopSrv>(tobas::kROSBagRecordStopSrv, &self::stopCb, this);
  clean_srv_ = createService<CleanSrv>(tobas::kROSBagCleanSrv, &self::cleanCb, this);
}

void ROSBagRecorderNode::imuCb(const tobas_msgs::msg::Imu::ConstSharedPtr& msg)
{
  if (!is_recording_)
    return;

  try
  {
    writer_.write(*msg, ns_ + tobas::kImuTopic, msg->header.stamp);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Failed to write \"" << tobas::kImuTopic << "\": " << e.what());
  }
}

void ROSBagRecorderNode::startCb(const StartSrv::Request::ConstSharedPtr& req, const StartSrv::Response::SharedPtr& res)
{
  if (is_recording_)
  {
    res->success = false;
    res->message = "Rosbag recording is in progress.";
    return;
  }

  if (req->name.empty())
  {
    res->success = false;
    res->message = "Please specify rosbag name.";
    return;
  }

  // ディレクトリのサイズが大きすぎないか確認
  const auto rosbag_dir_size = path::computeDirectorySize(rosbag_dir_);
  if (rosbag_dir_size > kMaxDirSize)
  {
    res->success = false;
    res->message = std::format(
      "The size of rosbag directory ({}) is over {} GB. Please clean it first.", rosbag_dir_.string(),
      kMaxDirSize / 1'000'000'000);
    return;
  }

  const auto uri = rosbag_dir_ / req->name;
  if (fs::exists(uri))
  {
    if (req->overwrite)
    {
      fs::remove_all(uri);
    }
    else
    {
      res->success = false;
      res->message = uri.string() + " already exists.";
      return;
    }
  }

  rosbag2_storage::StorageOptions options;
  options.uri = uri;
  options.max_bagfile_size = req->max_file_size;
  options.max_cache_size = req->max_cache_size;

  try
  {
    writer_.open(options);
  }
  catch (const std::exception& e)
  {
    res->success = false;
    res->message = "Failed to open " + options.uri + ": " + e.what();
    return;
  }

  is_recording_ = true;
  TOBAS_INFO("Rosbag recording has started.");

  res->success = true;
  res->message.clear();
}

void ROSBagRecorderNode::stopCb(const StopSrv::Request::ConstSharedPtr&, const StopSrv::Response::SharedPtr& res)
{
  if (!is_recording_)
  {
    res->success = false;
    res->message = "Rosbag recording is not in progress.";
    return;
  }

  try
  {
    writer_.close();
  }
  catch (const std::exception& e)
  {
    res->success = false;
    res->message = "Failed to close rosbag file: " + std::string(e.what());
    return;
  }

  is_recording_ = false;
  TOBAS_INFO("Rosbag recording has stopped.");

  res->success = true;
  res->message.clear();
}

void ROSBagRecorderNode::cleanCb(const CleanSrv::Request::ConstSharedPtr&, const CleanSrv::Response::SharedPtr& res)
{
  path::clearDirectory(rosbag_dir_);

  res->success = true;
  res->message.clear();
}

RCLCPP_COMPONENTS_REGISTER_NODE(ROSBagRecorderNode)
