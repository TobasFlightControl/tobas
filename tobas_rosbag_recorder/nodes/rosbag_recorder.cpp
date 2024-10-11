#include <rosbag2_cpp/writer.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tobas_path_tools/core.hpp>
#include <tobas_ros2_tools/filesystem.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>

#include <tobas_std_msgs/msg/message.hpp>
#include <tobas_kdl_msgs/msg/euler.hpp>
#include <tobas_kdl_msgs/msg/tree.hpp>
#include <tobas_drone_msgs/msg/drone.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/control_surface_deflections.hpp>
#include <tobas_msgs/msg/cpu.hpp>
#include <tobas_msgs/msg/event.hpp>
#include <tobas_msgs/msg/gps.hpp>
#include <tobas_msgs/msg/imu.hpp>
#include <tobas_msgs/msg/joint_command_array.hpp>
#include <tobas_msgs/msg/latency.hpp>
#include <tobas_msgs/msg/link_state_array.hpp>
#include <tobas_msgs/msg/magnetic_field.hpp>
#include <tobas_msgs/msg/odometry.hpp>
#include <tobas_msgs/msg/pose_twist_accel_command.hpp>
#include <tobas_msgs/msg/position_yaw.hpp>
#include <tobas_msgs/msg/pos_vel_acc_yaw.hpp>
#include <tobas_msgs/msg/pre_arm_check.hpp>
#include <tobas_msgs/msg/pwm_array.hpp>
#include <tobas_msgs/msg/rc_input.hpp>
#include <tobas_msgs/msg/roll_pitch_yaw_throttle.hpp>
#include <tobas_msgs/msg/rotor_speeds.hpp>
#include <tobas_msgs/msg/speed_roll_delta_pitch.hpp>
#include <tobas_msgs/msg/throttle_array.hpp>
#include <tobas_msgs/msg/wind.hpp>

#include <tobas_msgs/srv/bag_record_start.hpp>
#include <tobas_msgs/srv/bag_record_stop.hpp>

using namespace std;
namespace fs = filesystem;

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
  const string ns_;
  const fs::path rosbag_dir_;

  rosbag2_cpp::Writer writer_;
  bool is_recording_ = false;

  vector<rclcpp::SubscriptionBase::SharedPtr> subs_;

  ros2::ServiceServerPtr<StartSrv> start_srv_;
  ros2::ServiceServerPtr<StopSrv> stop_srv_;
  ros2::ServiceServerPtr<CleanSrv> clean_srv_;

  template <typename MsgType>
  void addSubscription(const char* topic, bool latch = false, bool reliable = false, size_t queue_size = 1);

  template <typename MsgType>
  void callback(const typename MsgType::ConstSharedPtr& msg, const char* topic);

  void startCb(const StartSrv::Request::ConstSharedPtr& req, const StartSrv::Response::SharedPtr& res);
  void stopCb(const StopSrv::Request::ConstSharedPtr& req, const StopSrv::Response::SharedPtr& res);
  void cleanCb(const CleanSrv::Request::ConstSharedPtr& req, const CleanSrv::Response::SharedPtr& res);
};

ROSBagRecorderNode::ROSBagRecorderNode(const rclcpp::NodeOptions& options)
  : super("rosbag_recorder", options),
    ns_(string(get_namespace()) + "/"),
    rosbag_dir_(ros2::expandUser(tobas::kROSBagDir))
{
  // Register subscriptions
  addSubscription<tobas_std_msgs::msg::Message>(tobas::kMessageTopic);
  addSubscription<tobas_drone_msgs::msg::Drone>(tobas::kDroneTopic, true, true);
  addSubscription<tobas_kdl_msgs::msg::Tree>(tobas::kKDLTreeTopic, true, true);
  addSubscription<std_msgs::msg::String>(tobas::kRobotDescriptionTopic, true, true);
  addSubscription<tobas_msgs::msg::Battery>(tobas::kBatteryTopic);
  addSubscription<tobas_msgs::msg::Cpu>(tobas::kCPUTopic);
  addSubscription<tobas_msgs::msg::RCInput>(tobas::kRcInputTopic);
  addSubscription<tobas_msgs::msg::Imu>(tobas::kIMUTopic);
  addSubscription<tobas_msgs::msg::MagneticField>(tobas::kMagTopic);
  addSubscription<sensor_msgs::msg::FluidPressure>(tobas::kAirPressureTopic);
  addSubscription<tobas_msgs::msg::Gps>(tobas::kGNSSTopic);
  addSubscription<tobas_msgs::msg::RotorSpeeds>(tobas::kRotorSpeedsTopic);
  addSubscription<sensor_msgs::msg::JointState>(tobas::kJointStatesTopic);
  addSubscription<tobas_msgs::msg::Odometry>(tobas::kOdometryTopic);
  addSubscription<tobas_kdl_msgs::msg::Euler>(tobas::kEulerTopic);
  addSubscription<tobas_msgs::msg::Wind>(tobas::kWindTopic);
  addSubscription<tobas_msgs::msg::Event>(tobas::kEventTopic);
  addSubscription<tobas_msgs::msg::Latency>(tobas::kLatencyTopic);
  addSubscription<std_msgs::msg::Bool>(tobas::kArmingTopic);
  addSubscription<tobas_msgs::msg::PreArmCheck>(tobas::kPreArmCheckTopic);
  addSubscription<tobas_msgs::msg::ThrottleArray>(tobas::kThrottlesCmdTopic);
  addSubscription<tobas_msgs::msg::RotorSpeeds>(tobas::kRotorSpeedsCmdTopic);
  addSubscription<tobas_msgs::msg::ControlSurfaceDeflections>(tobas::kDeflectionCmdTopic);
  addSubscription<tobas_msgs::msg::PwmArray>(tobas::kPwmCmdTopic);
  addSubscription<tobas_msgs::msg::PosVelAccYaw>(tobas::kPosVelAccYawCmdTopic);
  addSubscription<tobas_msgs::msg::PositionYaw>(tobas::kPositionYawCmdTopic);
  addSubscription<tobas_msgs::msg::RollPitchYawThrottle>(tobas::kRPYThrotCmdTopic);
  addSubscription<tobas_msgs::msg::PoseTwistAccelCommand>(tobas::kPoseTwistAccelCmdTopic);
  addSubscription<tobas_msgs::msg::SpeedRollDeltaPitch>(tobas::kSpeedRollDpitchCmdTopic);
  addSubscription<tobas_msgs::msg::JointCommandArray>(tobas::kJointPositionsCmdTopic);
  addSubscription<tobas_msgs::msg::JointCommandArray>(tobas::kJointVelocitiesCmdTopic);
  addSubscription<tobas_msgs::msg::JointCommandArray>(tobas::kJointEffortsCmdTopic);

  // Register services
  start_srv_ = createService<StartSrv>(tobas::kROSBagRecordStartSrv, &self::startCb, this);
  stop_srv_ = createService<StopSrv>(tobas::kROSBagRecordStopSrv, &self::stopCb, this);
  clean_srv_ = createService<CleanSrv>(tobas::kROSBagCleanSrv, &self::cleanCb, this);
}

template <typename MsgType>
void ROSBagRecorderNode::addSubscription(const char* topic, bool latch, bool reliable, size_t queue_size)
{
  const auto qos = ros2::makeQoS(latch, reliable, queue_size);
  const auto cb = [this, topic](const typename MsgType::ConstSharedPtr& msg) { callback<MsgType>(msg, topic); };
  const auto sub = create_subscription<MsgType>(topic, qos, cb);
  subs_.push_back(sub);
}

template <typename MsgType>
void ROSBagRecorderNode::callback(const typename MsgType::ConstSharedPtr& msg, const char* topic)
{
  if (!is_recording_)
    return;

  try
  {
    writer_.write(*msg, ns_ + topic, get_clock()->now());
  }
  catch (const exception& e)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Failed to write \"" << topic << "\": " << e.what());
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
    res->message = format(
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
  catch (const exception& e)
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
  catch (const exception& e)
  {
    res->success = false;
    res->message = "Failed to close rosbag file: " + string(e.what());
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
