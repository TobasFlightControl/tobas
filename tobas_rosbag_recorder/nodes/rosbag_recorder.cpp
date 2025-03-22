#include <rosbag2_cpp/writer.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tobas_path_tools/core.hpp>
#include <tobas_linux/core.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>

#include <tobas_std_msgs/msg/message.hpp>
#include <tobas_kdl_msgs_adapter/tree.hpp>
#include <tobas_kdl_msgs_adapter/wrench_stamped.hpp>
#include <tobas_msgs/msg/rosbag_state.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/control_surface_deflections.hpp>
#include <tobas_msgs/msg/cpu.hpp>
#include <tobas_msgs/msg/fluid_pressure_stamped.hpp>
#include <tobas_msgs/msg/fluid_pressure_with_variance_stamped.hpp>
#include <tobas_msgs/msg/ice_propulsion_system_command.hpp>
#include <tobas_msgs/msg/joint_command_array.hpp>
#include <tobas_msgs/msg/joint_state_array.hpp>
#include <tobas_msgs/msg/latency.hpp>
#include <tobas_msgs/msg/pre_arm_check.hpp>
#include <tobas_msgs/msg/pwm_array.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_msgs/msg/rotor_liveliness_array.hpp>
#include <tobas_msgs/msg/rotor_thrust_array.hpp>
#include <tobas_msgs_adapter/gnss.hpp>
#include <tobas_msgs_adapter/imu_stamped.hpp>
#include <tobas_msgs_adapter/imu_with_covariance_stamped.hpp>
#include <tobas_msgs_adapter/magnetic_field_stamped.hpp>
#include <tobas_msgs_adapter/magnetic_field_with_covariance_stamped.hpp>
#include <tobas_msgs_adapter/odometry.hpp>
#include <tobas_msgs_adapter/rc_input.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_debug_msgs_adapter/observer_feedback.hpp>
#include <tobas_debug_msgs_adapter/multi_rotor_controller_feedback.hpp>
#include <tobas_debug_msgs/msg/fixed_wing_controller_feedback.hpp>

#include <tobas_msgs/srv/bag_record_start.hpp>
#include <tobas_msgs/srv/bag_record_stop.hpp>

#define BILLION 1'000'000'000

using namespace std;
namespace fs = filesystem;

class RosbagRecorderNode : public tobas::BaseNode
{
  using self = RosbagRecorderNode;
  using super = tobas::BaseNode;

  using StartSrv = tobas_msgs::srv::BagRecordStart;
  using StopSrv = tobas_msgs::srv::BagRecordStop;
  using CleanSrv = std_srvs::srv::Trigger;

  static constexpr size_t kMaxParDirSize = 10UL * BILLION;  // [byte]
  static constexpr auto kMainTimerPeriod = 1s;

public:
  explicit RosbagRecorderNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  const string ns_;
  const fs::path rosbag_dir_;

  rosbag2_cpp::Writer writer_;
  fs::path file_path_;
  bool recording_ = false;
  rclcpp::Time start_time_;
  size_t msg_cnt_;

  // ROS message buffers
  tobas_drone_msgs::msg::Drone drone_;
  tobas_kdl_msgs::msg::Tree tree_;
  tobas_msgs::msg::RCInput rcin_;
  tobas_msgs::msg::ImuWithCovarianceStamped imu_;
  tobas_msgs::msg::ImuStamped imu_raw_;
  tobas_msgs::msg::MagneticFieldWithCovarianceStamped mag_;
  tobas_msgs::msg::MagneticFieldStamped mag_raw_;
  tobas_msgs::msg::Gnss gnss_;
  tobas_msgs::msg::Odometry odom_;
  tobas_kdl_msgs::msg::WrenchStamped dist_force_;
  tobas_debug_msgs::msg::ObserverFeedback obsv_fb_;
  tobas_debug_msgs::msg::MultiRotorControllerFeedback mr_ctrl_fb_;

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::RosbagState> rosbag_state_pub_;

  // Subscribers
  vector<rclcpp::SubscriptionBase::SharedPtr> subs_;

  // Service servers
  ros2::ServiceServerPtr<StartSrv> start_srv_;
  ros2::ServiceServerPtr<StopSrv> stop_srv_;
  ros2::ServiceServerPtr<CleanSrv> clean_srv_;

  // Timers
  ros2::TimerPtr main_timer_;

  void publishRosbagState();

  template <typename MsgType>
  inline void write(const MsgType& msg, const char* topic) noexcept;

  template <typename MsgType>
  void addStandardMsgSub(
    const char* topic,
    bool latch = ros2::qos::kDefaultLatch,
    bool reliable = ros2::qos::kDefaultReliable,
    size_t queue_size = ros2::qos::kDefaultQueueSize);

  template <typename ExtMsgType, typename RawMsgType>
  void addTypeAdaptedMsgSub(
    RawMsgType& raw_msg,
    const char* topic,
    bool latch = ros2::qos::kDefaultLatch,
    bool reliable = ros2::qos::kDefaultReliable,
    size_t queue_size = ros2::qos::kDefaultQueueSize);

  template <typename MsgType>
  void standardMsgCb(const typename MsgType::ConstSharedPtr& msg, const char* topic);

  template <typename ExtMsgType, typename RawMsgType>
  void typeAdaptedMsgCb(const typename ExtMsgType::ConstSharedPtr& ext_msg, RawMsgType& raw_msg, const char* topic);

  void startCb(const StartSrv::Request::ConstSharedPtr& req, const StartSrv::Response::SharedPtr& res);
  void stopCb(const StopSrv::Request::ConstSharedPtr& req, const StopSrv::Response::SharedPtr& res);
  void cleanCb(const CleanSrv::Request::ConstSharedPtr& req, const CleanSrv::Response::SharedPtr& res);

  void mainTimerCb();
};

RosbagRecorderNode::RosbagRecorderNode(const rclcpp::NodeOptions& options)
  : super("rosbag_recorder", options),
    ns_(string(get_namespace()) + "/"),
    rosbag_dir_(linux::isSuperUser() ? tobas::kRosbagDirRoot : ros2::expandUser(tobas::kRosbagDirHome))
{
  // XXX: トピック通信の接続はローカルであっても遅延の原因になりうるため，レコード開始時ではなく先に接続を確立しておく．

  rosbag_state_pub_ = createPublisher<tobas_msgs::msg::RosbagState>(tobas::kRosbagStateTopic);

  // Resister subscribers for standard messages
  addStandardMsgSub<tobas_std_msgs::msg::Message>(tobas::kMessageTopic);
  addStandardMsgSub<std_msgs::msg::String>(tobas::kRobotDescriptionTopic, true, true);
  addStandardMsgSub<tobas_msgs::msg::Battery>(tobas::kBatteryTopic);
  addStandardMsgSub<tobas_msgs::msg::Cpu>(tobas::kCpuTopic);
  addStandardMsgSub<tobas_msgs::msg::FluidPressureWithVarianceStamped>(tobas::kAirPressureTopic);
  addStandardMsgSub<tobas_msgs::msg::FluidPressureStamped>(tobas::kAirPressureRawTopic);
  addStandardMsgSub<tobas_msgs::msg::RotorStateArray>(tobas::kRotorStatesTopic);
  addStandardMsgSub<tobas_msgs::msg::RotorLivelinessArray>(tobas::kRotorLivelinessTopic);
  addStandardMsgSub<tobas_msgs::msg::JointStateArray>(tobas::kJointStatesTopic);
  addStandardMsgSub<tobas_msgs::msg::Latency>(tobas::kImuSamplingTimeTopic);
  addStandardMsgSub<tobas_msgs::msg::Latency>(tobas::kControlLatencyTopic);
  addStandardMsgSub<tobas_msgs::msg::Arming>(tobas::kArmingTopic);
  addStandardMsgSub<tobas_msgs::msg::PreArmCheck>(tobas::kPreArmCheckTopic);
  addStandardMsgSub<tobas_msgs::msg::RotorThrustArray>(tobas::kRotorThrustsCmdTopic);
  addStandardMsgSub<tobas_msgs::msg::RotorSpeedArray>(tobas::kRotorSpeedsCmdTopic);
  addStandardMsgSub<tobas_msgs::msg::IcePropulsionSystemCommand>(tobas::kIcePropulsionSystemCmdTopic);
  addStandardMsgSub<tobas_msgs::msg::ControlSurfaceDeflections>(tobas::kDeflectionCmdTopic);
  addStandardMsgSub<tobas_msgs::msg::PwmArray>(tobas::kPwmCmdTopic);
  addStandardMsgSub<tobas_msgs::msg::JointCommandArray>(tobas::kJointPosCmdTopic);
  addStandardMsgSub<tobas_msgs::msg::JointCommandArray>(tobas::kJointVelCmdTopic);
  addStandardMsgSub<tobas_msgs::msg::JointCommandArray>(tobas::kJointEffCmdTopic);
  addStandardMsgSub<tobas_debug_msgs::msg::FixedWingControllerFeedback>(tobas::kFWCtrlFeedbackTopic);

  // Resister subscribers for non-standard messages
  addTypeAdaptedMsgSub<tobas::Drone>(drone_, tobas::kDroneTopic, true, true);
  addTypeAdaptedMsgSub<kdl::Tree>(tree_, tobas::kKdlTreeTopic, true, true);
  addTypeAdaptedMsgSub<tobas_msgs::RCInput>(rcin_, tobas::kRcInputTopic);
  addTypeAdaptedMsgSub<tobas_msgs::ImuWithCovarianceStamped>(imu_, tobas::kImuTopic);
  addTypeAdaptedMsgSub<tobas_msgs::ImuStamped>(imu_raw_, tobas::kImuRawTopic);
  addTypeAdaptedMsgSub<tobas_msgs::MagneticFieldWithCovarianceStamped>(mag_, tobas::kMagTopic);
  addTypeAdaptedMsgSub<tobas_msgs::MagneticFieldStamped>(mag_raw_, tobas::kMagRawTopic);
  addTypeAdaptedMsgSub<tobas_msgs::Gnss>(gnss_, tobas::kGnssTopic);
  addTypeAdaptedMsgSub<tobas_msgs::Odometry>(odom_, tobas::kOdometryTopic);
  addTypeAdaptedMsgSub<tobas_kdl_msgs::WrenchStamped>(dist_force_, tobas::kDisturbanceForceTopic);
  addTypeAdaptedMsgSub<tobas_debug_msgs::ObserverFeedback>(obsv_fb_, tobas::kObsvFeedbackTopic);
  addTypeAdaptedMsgSub<tobas_debug_msgs::MultiRotorControllerFeedback>(mr_ctrl_fb_, tobas::kMRCtrlFeedbackTopic);

  // Register services
  start_srv_ = createService<StartSrv>(tobas::kRosbagRecordStartSrv, &self::startCb, this);
  stop_srv_ = createService<StopSrv>(tobas::kRosbagRecordStopSrv, &self::stopCb, this);
  clean_srv_ = createService<CleanSrv>(tobas::kRosbagCleanSrv, &self::cleanCb, this);

  main_timer_ = createTimer(kMainTimerPeriod, &self::mainTimerCb, this);
}

void RosbagRecorderNode::publishRosbagState()
{
  const auto now = get_clock()->now();

  auto rosbag_state = std::make_unique<tobas_msgs::msg::RosbagState>();
  rosbag_state->header.stamp = now;
  rosbag_state->recording = recording_;

  if (recording_)
  {
    const auto file_size = path::computeDirectorySize(file_path_);

    rosbag_state->file_path = file_path_;
    rosbag_state->duration = now - start_time_;
    rosbag_state->file_size = file_size;
    rosbag_state->message_count = msg_cnt_;

    if (file_size > tobas::kMaxRosbagSize)
    {
      try
      {
        writer_.close();
      }
      catch (const exception& e)
      {
        TOBAS_ERROR("Failed to close rosbag file: ", e.what());
        return;
      }

      recording_ = false;

      TOBAS_WARN(
        "The recording is terminated because the size of rosbag ", file_path_, " exceeded ",
        tobas::kMaxRosbagSize / BILLION, "GB.");
    }
  }

  rosbag_state_pub_->publish(move(rosbag_state));
}

template <typename MsgType>
inline void RosbagRecorderNode::write(const MsgType& msg, const char* topic) noexcept
{
  try
  {
    writer_.write(msg, ns_ + topic, get_clock()->now());
  }
  catch (const exception& e)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Failed to write \"" << topic << "\": " << e.what());
    return;
  }

  ++msg_cnt_;
}

template <typename MsgType>
void RosbagRecorderNode::addStandardMsgSub(const char* topic, bool latch, bool reliable, size_t queue_size)
{
  const auto qos = ros2::makeQoS(latch, reliable, queue_size);
  const auto cb = [this, topic](const typename MsgType::ConstSharedPtr& msg) { standardMsgCb<MsgType>(msg, topic); };
  const auto sub = create_subscription<MsgType>(topic, qos, cb);
  subs_.push_back(sub);
}

template <typename ExtMsgType, typename RawMsgType>
void RosbagRecorderNode::addTypeAdaptedMsgSub(
  RawMsgType& raw_msg,
  const char* topic,
  bool latch,
  bool reliable,
  size_t queue_size)
{
  const auto qos = ros2::makeQoS(latch, reliable, queue_size);
  const auto cb = [this, &raw_msg, topic](const typename ExtMsgType::ConstSharedPtr& ext_msg)
  { typeAdaptedMsgCb<ExtMsgType, RawMsgType>(ext_msg, raw_msg, topic); };
  const auto sub = create_subscription<ExtMsgType>(topic, qos, cb);
  subs_.push_back(sub);
}

template <typename MsgType>
void RosbagRecorderNode::standardMsgCb(const typename MsgType::ConstSharedPtr& msg, const char* topic)
{
  if (!recording_)
    return;

  this->write(*msg, topic);
}

template <typename ExtMsgType, typename RawMsgType>
void RosbagRecorderNode::typeAdaptedMsgCb(
  const typename ExtMsgType::ConstSharedPtr& ext_msg,
  RawMsgType& raw_msg,
  const char* topic)
{
  if (!recording_)
    return;

  // Publisher側にシリアライズさせるのを防ぐため，TypeAdapterのまま購読し，こちら側でROSメッセージへの変換を行う．
  rclcpp::TypeAdapter<ExtMsgType, RawMsgType>::convert_to_ros_message(*ext_msg, raw_msg);

  this->write(raw_msg, topic);
}

void RosbagRecorderNode::startCb(const StartSrv::Request::ConstSharedPtr& req, const StartSrv::Response::SharedPtr& res)
{
  if (recording_)
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

  // rosbagディレクトリが存在しなければ作成
  if (!fs::exists(rosbag_dir_))
    fs::create_directories(rosbag_dir_);

  // rosbagディレクトリ全体のサイズが大きすぎないか確認
  const auto par_dir_size = path::computeDirectorySize(rosbag_dir_);
  if (par_dir_size > kMaxParDirSize)
  {
    res->success = false;
    res->message = "The size of rosbag directory (" + rosbag_dir_.string() + ") is over "
                   + to_string(kMaxParDirSize / BILLION) + " GB. Please clean it first.";
    return;
  }

  file_path_ = rosbag_dir_ / req->name;
  if (fs::exists(file_path_))
  {
    if (req->overwrite)
    {
      fs::remove_all(file_path_);
    }
    else
    {
      res->success = false;
      res->message = file_path_.string() + " already exists.";
      return;
    }
  }

  rosbag2_storage::StorageOptions options;
  options.uri = file_path_;
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

  recording_ = true;
  start_time_ = get_clock()->now();
  msg_cnt_ = 0;

  TOBAS_INFO("Rosbag recording has started.");

  res->success = true;
  res->message.clear();

  publishRosbagState();
}

void RosbagRecorderNode::stopCb(const StopSrv::Request::ConstSharedPtr&, const StopSrv::Response::SharedPtr& res)
{
  if (!recording_)
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

  recording_ = false;

  TOBAS_INFO("Rosbag recording has stopped.");

  res->success = true;
  res->message.clear();

  publishRosbagState();
}

void RosbagRecorderNode::cleanCb(const CleanSrv::Request::ConstSharedPtr&, const CleanSrv::Response::SharedPtr& res)
{
  path::clearDirectory(rosbag_dir_);

  res->success = true;
  res->message.clear();
}

void RosbagRecorderNode::mainTimerCb()
{
  publishRosbagState();
}

RCLCPP_COMPONENTS_REGISTER_NODE(RosbagRecorderNode)
