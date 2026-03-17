#include <rosbag2_cpp/writer.hpp>

#include <tobas_constants/path.hpp>
#include <tobas_constants/ros_interface.hpp>
#include <tobas_constants/rosbag.hpp>
#include <tobas_linux/core.hpp>
#include <tobas_node/node.hpp>
#include <tobas_path_tools/core.hpp>
#include <tobas_ros2_tools/util.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tobas_debug_msgs/msg/fixed_wing_controller_feedback.hpp>
#include <tobas_debug_msgs_adapter/multicopter_controller_feedback.hpp>
#include <tobas_debug_msgs_adapter/observer_feedback.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_kdl_msgs_adapter/tree.hpp>
#include <tobas_kdl_msgs_adapter/wrench_stamped.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/cpu.hpp>
#include <tobas_msgs/msg/fluid_pressure.hpp>
#include <tobas_msgs/msg/ice_propulsion_system_command.hpp>
#include <tobas_msgs/msg/joint_command_array.hpp>
#include <tobas_msgs/msg/joint_state_array.hpp>
#include <tobas_msgs/msg/latency.hpp>
#include <tobas_msgs/msg/message.hpp>
#include <tobas_msgs/msg/pwm_array.hpp>
#include <tobas_msgs/msg/rosbag_state.hpp>
#include <tobas_msgs/msg/rotor_liveliness_array.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_msgs/msg/rotor_thrust_array.hpp>
#include <tobas_msgs/msg/vehicle_health.hpp>
#include <tobas_msgs/srv/bag_record_start.hpp>
#include <tobas_msgs/srv/bag_record_stop.hpp>
#include <tobas_msgs_adapter/gnss.hpp>
#include <tobas_msgs_adapter/imu.hpp>
#include <tobas_msgs_adapter/magnetic_field.hpp>
#include <tobas_msgs_adapter/odometry_with_covariance_stamped.hpp>
#include <tobas_msgs_adapter/rc_input.hpp>
#include <tobas_msgs_adapter/vibration_level.hpp>

#define BILLION 1'000'000'000UL

using namespace std::chrono_literals;
namespace fs = std::filesystem;

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
  const std::string ns_;
  const fs::path rosbag_dir_;

  rosbag2_cpp::Writer writer_;
  fs::path file_path_;
  bool recording_ = false;
  rclcpp::Time start_time_;
  size_t msg_cnt_;

  // ROS standard message buffers
  tobas_drone_msgs::msg::Drone drone_;
  tobas_kdl_msgs::msg::Tree tree_;
  tobas_msgs::msg::RCInput rcin_;
  tobas_msgs::msg::Imu imu_;
  tobas_msgs::msg::MagneticField mag_;
  tobas_msgs::msg::Gnss gnss_;
  tobas_msgs::msg::OdometryWithCovarianceStamped odom_;
  tobas_msgs::msg::VibrationLevel vibe_;
  tobas_kdl_msgs::msg::WrenchStamped dist_force_;
  tobas_debug_msgs::msg::ObserverFeedback obsv_fb_;
  tobas_debug_msgs::msg::MulticopterControllerFeedback mr_ctrl_fb_;

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::RosbagState> rosbag_state_pub_;

  // Subscribers
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;

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
  : super("rosbag_recorder", options)
  , ns_(std::string(get_namespace()) + "/")
  , rosbag_dir_(linux::isSuperUser() ? tobas::kRosbagDirRoot : ros2::expandUser(tobas::kRosbagDirHome))
{
  // Register publishers
  rosbag_state_pub_ = createPublisher<tobas_msgs::msg::RosbagState>(tobas::topic::kRosbagState);

  // Resister subscribers
  // トピック通信の接続はローカルであっても遅延の原因になりうるため，レコード開始時ではなく先に接続を確立しておく．
  addStandardMsgSub<tobas_msgs::msg::Message>(tobas::topic::kMessage);
  addTypeAdaptedMsgSub<tobas::Drone>(drone_, tobas::topic::kDrone, true, true);
  addTypeAdaptedMsgSub<kdl::Tree>(tree_, tobas::topic::kKdlTree, true, true);
  addStandardMsgSub<std_msgs::msg::String>(tobas::topic::kRobotDescription, true, true);
  addStandardMsgSub<tobas_msgs::msg::Battery>(tobas::topic::kBattery);
  addStandardMsgSub<tobas_msgs::msg::Cpu>(tobas::topic::kCpu);
  addTypeAdaptedMsgSub<tobas_msgs::RCInput>(rcin_, tobas::topic::kRcInput);
  addTypeAdaptedMsgSub<tobas_msgs::Imu>(imu_, tobas::topic::kImuRaw);
  addTypeAdaptedMsgSub<tobas_msgs::Imu>(imu_, tobas::topic::kImuFilt);
  addTypeAdaptedMsgSub<tobas_msgs::MagneticField>(mag_, tobas::topic::kMagneticField);
  addStandardMsgSub<tobas_msgs::msg::FluidPressure>(tobas::topic::kAirPressure);
  addTypeAdaptedMsgSub<tobas_msgs::Gnss>(gnss_, tobas::topic::kGnss);
  addStandardMsgSub<tobas_msgs::msg::RotorStateArray>(tobas::topic::kRotorStates);
  addStandardMsgSub<tobas_msgs::msg::RotorLivelinessArray>(tobas::topic::kRotorLiv);
  addStandardMsgSub<tobas_msgs::msg::JointStateArray>(tobas::topic::kJointStates);
  addTypeAdaptedMsgSub<tobas_msgs::OdometryWithCovarianceStamped>(odom_, tobas::topic::kOdometry);
  addStandardMsgSub<tobas_msgs::msg::Latency>(tobas::topic::kImuSamplingTime);
  addStandardMsgSub<tobas_msgs::msg::Latency>(tobas::topic::kControlLatency);
  addStandardMsgSub<tobas_msgs::msg::Arming>(tobas::topic::kArming);
  addStandardMsgSub<tobas_msgs::msg::VehicleHealth>(tobas::topic::kVehicleHealth);
  addTypeAdaptedMsgSub<tobas_msgs::VibrationLevel>(vibe_, tobas::topic::kVibrationLevel);
  addTypeAdaptedMsgSub<tobas_kdl_msgs::WrenchStamped>(dist_force_, tobas::topic::kDisturbanceForce);
  addStandardMsgSub<tobas_msgs::msg::RotorThrustArray>(tobas::topic::kRotorThrustsCmd);
  addStandardMsgSub<tobas_msgs::msg::RotorSpeedArray>(tobas::topic::kRotorSpeedsCmd);
  addStandardMsgSub<tobas_msgs::msg::IcePropulsionSystemCommand>(tobas::topic::kIcePropulsionSystemCmd);
  addStandardMsgSub<tobas_msgs::msg::PwmArray>(tobas::topic::kPwmCmd);
  addStandardMsgSub<tobas_msgs::msg::JointCommandArray>(tobas::topic::kJointPosCmd);
  addStandardMsgSub<tobas_msgs::msg::JointCommandArray>(tobas::topic::kJointVelCmd);
  addStandardMsgSub<tobas_msgs::msg::JointCommandArray>(tobas::topic::kJointEffCmd);
  addTypeAdaptedMsgSub<tobas_debug_msgs::ObserverFeedback>(obsv_fb_, tobas::topic::kObsvFeedback);
  addStandardMsgSub<tobas_debug_msgs::msg::FixedWingControllerFeedback>(tobas::topic::kFWCtrlFeedback);
  addTypeAdaptedMsgSub<tobas_debug_msgs::MulticopterControllerFeedback>(mr_ctrl_fb_, tobas::topic::kMRCtrlFeedback);

  // Register services
  start_srv_ = createService<StartSrv>(tobas::service::kRosbagRecordStart, &self::startCb, this);
  stop_srv_ = createService<StopSrv>(tobas::service::kRosbagRecordStop, &self::stopCb, this);
  clean_srv_ = createService<CleanSrv>(tobas::service::kRosbagClean, &self::cleanCb, this);

  // Start main timer
  main_timer_ = createTimer(kMainTimerPeriod, &self::mainTimerCb, this);
}

void RosbagRecorderNode::publishRosbagState()
{
  const auto cur_time = now();

  auto rosbag_state = std::make_unique<tobas_msgs::msg::RosbagState>();
  rosbag_state->header.stamp = cur_time;
  rosbag_state->recording = recording_;

  if (recording_) {
    const auto file_size = path::computeDirectorySize(file_path_);

    rosbag_state->file_path = file_path_;
    rosbag_state->duration = cur_time - start_time_;
    rosbag_state->file_size = file_size;
    rosbag_state->message_count = msg_cnt_;

    if (file_size > tobas::kMaxRosbagSize) {
      try {
        writer_.close();
      }
      catch (const std::exception& e) {
        TOBAS_ERROR("Failed to close rosbag file: ", e.what());
        return;
      }

      recording_ = false;

      TOBAS_WARN(
        "The recording is terminated because the size of rosbag ",
        file_path_,
        " exceeded ",
        tobas::kMaxRosbagSize / BILLION,
        "GB.");
    }
  }

  rosbag_state_pub_->publish(std::move(rosbag_state));
}

template <typename MsgType>
inline void RosbagRecorderNode::write(const MsgType& msg, const char* topic) noexcept
{
  try {
    writer_.write(msg, ns_ + topic, now());
  }
  catch (const std::exception& e) {
    TOBAS_ERROR("Failed to write \"", topic, "\": ", e.what());
    return;
  }

  ++msg_cnt_;
}

template <typename MsgType>
void RosbagRecorderNode::addStandardMsgSub(const char* topic, bool latch, bool reliable, size_t queue_size)
{
  const ros2::qos::QoS qos(latch, reliable, queue_size);
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
  const ros2::qos::QoS qos(latch, reliable, queue_size);
  const auto cb = [this, &raw_msg, topic](const typename ExtMsgType::ConstSharedPtr& ext_msg)
  { typeAdaptedMsgCb<ExtMsgType, RawMsgType>(ext_msg, raw_msg, topic); };
  const auto sub = create_subscription<ExtMsgType>(topic, qos, cb);
  subs_.push_back(sub);
}

template <typename MsgType>
void RosbagRecorderNode::standardMsgCb(const typename MsgType::ConstSharedPtr& msg, const char* topic)
{
  if (!recording_) {
    return;
  }

  this->write(*msg, topic);
}

template <typename ExtMsgType, typename RawMsgType>
void RosbagRecorderNode::typeAdaptedMsgCb(
  const typename ExtMsgType::ConstSharedPtr& ext_msg,
  RawMsgType& raw_msg,
  const char* topic)
{
  if (!recording_) {
    return;
  }

  // Publisher側にシリアライズさせるのを防ぐため，TypeAdapterのまま購読し，こちら側でROSメッセージへの変換を行う．
  rclcpp::TypeAdapter<ExtMsgType, RawMsgType>::convert_to_ros_message(*ext_msg, raw_msg);

  this->write(raw_msg, topic);
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

  // rosbagディレクトリが存在しなければ作成
  if (!fs::exists(rosbag_dir_)) {
    fs::create_directories(rosbag_dir_);
  }

  // rosbagディレクトリ全体のサイズが大きすぎないか確認
  const auto par_dir_size = path::computeDirectorySize(rosbag_dir_);
  if (par_dir_size > kMaxParDirSize) {
    res->success = false;
    res->message = "The size of rosbag directory (" + rosbag_dir_.string() + ") is over " +
                   std::to_string(kMaxParDirSize / BILLION) + " GB. Please clean it first.";
    return;
  }

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

  rosbag2_storage::StorageOptions options;
  options.uri = file_path_;
  options.max_bagfile_size = req->max_file_size;
  options.max_cache_size = req->max_cache_size;

  try {
    writer_.open(options);
  }
  catch (const std::exception& e) {
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

RCLCPP_COMPONENTS_REGISTER_NODE(RosbagRecorderNode)
