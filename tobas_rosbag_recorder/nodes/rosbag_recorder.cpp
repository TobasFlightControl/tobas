#include <rosbag2_cpp/writer.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tobas_path_tools/core.hpp>
#include <tobas_linux/core.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>

#include <tobas_std_msgs/msg/message.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/control_surface_deflections.hpp>
#include <tobas_msgs/msg/cpu.hpp>
#include <tobas_msgs/msg/event.hpp>
#include <tobas_msgs/msg/joint_command_array.hpp>
#include <tobas_msgs/msg/latency.hpp>
#include <tobas_msgs/msg/pre_arm_check.hpp>
#include <tobas_msgs/msg/pwm_array.hpp>
#include <tobas_msgs/msg/rc_input.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_msgs/msg/rotor_thrust_array.hpp>
#include <tobas_msgs/msg/speed_roll_delta_pitch.hpp>
#include <tobas_kdl_msgs_adapter/EulerStamped.hpp>
#include <tobas_kdl_msgs_adapter/Tree.hpp>
#include <tobas_drone_msgs_adapter/Drone.hpp>
#include <tobas_msgs_adapter/Gps.hpp>
#include <tobas_msgs_adapter/Imu.hpp>
#include <tobas_msgs_adapter/MagneticField.hpp>
#include <tobas_msgs_adapter/Odometry.hpp>
#include <tobas_msgs_adapter/PoseTwistAccelCommand.hpp>
#include <tobas_msgs_adapter/PosVelAccYaw.hpp>
#include <tobas_msgs_adapter/RollPitchYawThrottle.hpp>
#include <tobas_msgs_adapter/Wind.hpp>
#include <tobas_debug_msgs/msg/observer_feedback.hpp>
#include <tobas_debug_msgs/msg/multi_rotor_controller_feedback.hpp>
#include <tobas_debug_msgs/msg/non_planar_controller_feedback.hpp>
#include <tobas_debug_msgs/msg/fixed_wing_controller_feedback.hpp>

#include <tobas_msgs/srv/bag_record_start.hpp>
#include <tobas_msgs/srv/bag_record_stop.hpp>

#define BILLION 1'000'000'000

using namespace std;
namespace fs = filesystem;

class ROSBagRecorderNode : public tobas::BaseNode
{
  using self = ROSBagRecorderNode;
  using super = tobas::BaseNode;

  using StartSrv = tobas_msgs::srv::BagRecordStart;
  using StopSrv = tobas_msgs::srv::BagRecordStop;
  using CleanSrv = std_srvs::srv::Trigger;

  static constexpr size_t kMaxROSBagSize = 5UL * BILLION;   // [byte]
  static constexpr size_t kMaxParDirSize = 10UL * BILLION;  // [byte]
  static constexpr auto kCheckSizeTimerPeriod = 10s;

public:
  explicit ROSBagRecorderNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  const string ns_;
  const fs::path rosbag_dir_;

  rosbag2_cpp::Writer writer_;
  bool is_recording_ = false;
  fs::path rosbag_path_;

  // ROS message buffers
  tobas_drone_msgs::msg::Drone drone_;
  tobas_kdl_msgs::msg::Tree tree_;
  tobas_msgs::msg::Imu imu_;
  tobas_msgs::msg::MagneticField mag_;
  tobas_msgs::msg::Gps gps_;
  tobas_msgs::msg::Odometry odom_;
  tobas_kdl_msgs::msg::EulerStamped euler_;
  tobas_msgs::msg::Wind wind_;
  tobas_msgs::msg::PosVelAccYaw pvay_;
  tobas_msgs::msg::RollPitchYawThrottle rpyt_;
  tobas_msgs::msg::PoseTwistAccelCommand pta_;

  // Subscribers
  vector<rclcpp::SubscriptionBase::SharedPtr> subs_;

  // Service servers
  ros2::ServiceServerPtr<StartSrv> start_srv_;
  ros2::ServiceServerPtr<StopSrv> stop_srv_;
  ros2::ServiceServerPtr<CleanSrv> clean_srv_;

  // Timers
  ros2::TimerPtr check_size_timer_;

  void registerStandardMsgSub();
  void registerNonStandardMsgSub();
  void unregisterSubscriptions();

  template <typename MsgType>
  void addStandardMsgSub(
    const char* topic,
    bool latch = ros2::qos::kDefaultLatch,
    bool reliable = ros2::qos::kDefaultReliable,
    size_t queue_size = ros2::qos::kDefaultQueueSize);

  template <typename MsgType>
  void callback(const typename MsgType::ConstSharedPtr& msg, const char* topic);

  // Publisher側にシリアライズさせるのを防ぐため，TypeAdapterのまま購読し，こちら側でROSメッセージへの変換を行う．
  void droneCb(const tobas::Drone::ConstSharedPtr& msg);
  void treeCb(const kdl::Tree::ConstSharedPtr& msg);
  void imuCb(const tobas_msgs::Imu::ConstSharedPtr& msg);
  void magCb(const tobas_msgs::MagneticField::ConstSharedPtr& msg);
  void gnssCb(const tobas_msgs::Gps::ConstSharedPtr& msg);
  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& msg);
  void eulerCb(const tobas_kdl_msgs::EulerStamped::ConstSharedPtr& msg);
  void windCb(const tobas_msgs::Wind::ConstSharedPtr& msg);
  void posVelAccYawCmdCb(const tobas_msgs::PosVelAccYaw::ConstSharedPtr& msg);
  void rollPitchYawThrotCmdCb(const tobas_msgs::RollPitchYawThrottle::ConstSharedPtr& msg);
  void poseTwistAccelCmdCb(const tobas_msgs::PoseTwistAccelCommand::ConstSharedPtr& msg);

  void startCb(const StartSrv::Request::ConstSharedPtr& req, const StartSrv::Response::SharedPtr& res);
  void stopCb(const StopSrv::Request::ConstSharedPtr& req, const StopSrv::Response::SharedPtr& res);
  void cleanCb(const CleanSrv::Request::ConstSharedPtr& req, const CleanSrv::Response::SharedPtr& res);

  void checkSizeTimerCb();
};

ROSBagRecorderNode::ROSBagRecorderNode(const rclcpp::NodeOptions& options)
  : super("rosbag_recorder", options),
    ns_(string(get_namespace()) + "/"),
    rosbag_dir_(linux::isSuperUser() ? tobas::kROSBagDirRoot : linux::expandUser(tobas::kROSBagDirHome))
{
  // Register services
  start_srv_ = createService<StartSrv>(tobas::kROSBagRecordStartSrv, &self::startCb, this);
  stop_srv_ = createService<StopSrv>(tobas::kROSBagRecordStopSrv, &self::stopCb, this);
  clean_srv_ = createService<CleanSrv>(tobas::kROSBagCleanSrv, &self::cleanCb, this);

  check_size_timer_ = createTimer(kCheckSizeTimerPeriod, &self::checkSizeTimerCb, this, false);
}

void ROSBagRecorderNode::registerStandardMsgSub()
{
  addStandardMsgSub<tobas_std_msgs::msg::Message>(tobas::kMessageTopic);
  addStandardMsgSub<std_msgs::msg::String>(tobas::kRobotDescriptionTopic, true, true);
  addStandardMsgSub<tobas_msgs::msg::Battery>(tobas::kBatteryTopic);
  addStandardMsgSub<tobas_msgs::msg::Cpu>(tobas::kCPUTopic);
  addStandardMsgSub<tobas_msgs::msg::RCInput>(tobas::kRcInputTopic);
  addStandardMsgSub<sensor_msgs::msg::FluidPressure>(tobas::kAirPressureTopic);
  addStandardMsgSub<tobas_msgs::msg::RotorStateArray>(tobas::kRotorStatesTopic);
  addStandardMsgSub<sensor_msgs::msg::JointState>(tobas::kJointStatesTopic);
  addStandardMsgSub<tobas_msgs::msg::Event>(tobas::kEventTopic);
  addStandardMsgSub<tobas_msgs::msg::Latency>(tobas::kLatencyTopic);
  addStandardMsgSub<std_msgs::msg::Bool>(tobas::kArmingTopic);
  addStandardMsgSub<tobas_msgs::msg::PreArmCheck>(tobas::kPreArmCheckTopic);
  addStandardMsgSub<tobas_msgs::msg::RotorThrustArray>(tobas::kRotorThrustsCmdTopic);
  addStandardMsgSub<tobas_msgs::msg::RotorSpeedArray>(tobas::kRotorSpeedsCmdTopic);
  addStandardMsgSub<tobas_msgs::msg::ControlSurfaceDeflections>(tobas::kDeflectionCmdTopic);
  addStandardMsgSub<tobas_msgs::msg::PwmArray>(tobas::kPwmCmdTopic);
  addStandardMsgSub<tobas_msgs::msg::SpeedRollDeltaPitch>(tobas::kSpeedRollDpitchCmdTopic);
  addStandardMsgSub<tobas_msgs::msg::JointCommandArray>(tobas::kJointPositionsCmdTopic);
  addStandardMsgSub<tobas_msgs::msg::JointCommandArray>(tobas::kJointVelocitiesCmdTopic);
  addStandardMsgSub<tobas_msgs::msg::JointCommandArray>(tobas::kJointEffortsCmdTopic);
  addStandardMsgSub<tobas_debug_msgs::msg::ObserverFeedback>(tobas::kObsvFeedbackTopic);
  addStandardMsgSub<tobas_debug_msgs::msg::MultiRotorControllerFeedback>(tobas::kMRCtrlFeedbackTopic);
  addStandardMsgSub<tobas_debug_msgs::msg::NonPlanarControllerFeedback>(tobas::kNPCtrlFeedbackTopic);
  addStandardMsgSub<tobas_debug_msgs::msg::FixedWingControllerFeedback>(tobas::kFWCtrlFeedbackTopic);
}

void ROSBagRecorderNode::registerNonStandardMsgSub()
{
  subs_.push_back(createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true));
  subs_.push_back(createSubscriber(tobas::kKDLTreeTopic, &self::treeCb, this, true, true));
  subs_.push_back(createSubscriber(tobas::kIMUTopic, &self::imuCb, this));
  subs_.push_back(createSubscriber(tobas::kMagTopic, &self::magCb, this));
  subs_.push_back(createSubscriber(tobas::kGNSSTopic, &self::gnssCb, this));
  subs_.push_back(createSubscriber(tobas::kOdometryTopic, &self::odomCb, this));
  subs_.push_back(createSubscriber(tobas::kEulerTopic, &self::eulerCb, this));
  subs_.push_back(createSubscriber(tobas::kWindTopic, &self::windCb, this));
  subs_.push_back(createSubscriber(tobas::kPosVelAccYawCmdTopic, &self::posVelAccYawCmdCb, this));
  subs_.push_back(createSubscriber(tobas::kRPYThrotCmdTopic, &self::rollPitchYawThrotCmdCb, this));
  subs_.push_back(createSubscriber(tobas::kPoseTwistAccelCmdTopic, &self::poseTwistAccelCmdCb, this));
}

void ROSBagRecorderNode::unregisterSubscriptions()
{
  subs_.clear();
}

template <typename MsgType>
void ROSBagRecorderNode::addStandardMsgSub(const char* topic, bool latch, bool reliable, size_t queue_size)
{
  const auto qos = ros2::makeQoS(latch, reliable, queue_size);
  const auto cb = [this, topic](const typename MsgType::ConstSharedPtr& msg) { callback<MsgType>(msg, topic); };
  const auto sub = create_subscription<MsgType>(topic, qos, cb);
  subs_.push_back(sub);
}

template <typename MsgType>
void ROSBagRecorderNode::callback(const typename MsgType::ConstSharedPtr& msg, const char* topic)
{
  try
  {
    writer_.write(*msg, ns_ + topic, get_clock()->now());
  }
  catch (const exception& e)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Failed to write \"" << topic << "\": " << e.what());
  }
}

void ROSBagRecorderNode::droneCb(const tobas::Drone::ConstSharedPtr& msg)
{
  tobas_drone_msgs::DroneAdapter::convert_to_ros_message(*msg, drone_);

  try
  {
    writer_.write(drone_, ns_ + tobas::kDroneTopic, get_clock()->now());
  }
  catch (const exception& e)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Failed to write \"" << tobas::kDroneTopic << "\": " << e.what());
  }
}

void ROSBagRecorderNode::treeCb(const kdl::Tree::ConstSharedPtr& msg)
{
  tobas_kdl_msgs::TreeAdapter::convert_to_ros_message(*msg, tree_);

  try
  {
    writer_.write(tree_, ns_ + tobas::kKDLTreeTopic, get_clock()->now());
  }
  catch (const exception& e)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Failed to write \"" << tobas::kKDLTreeTopic << "\": " << e.what());
  }
}

void ROSBagRecorderNode::imuCb(const tobas_msgs::Imu::ConstSharedPtr& msg)
{
  tobas_msgs::ImuAdapter::convert_to_ros_message(*msg, imu_);

  try
  {
    writer_.write(imu_, ns_ + tobas::kIMUTopic, get_clock()->now());
  }
  catch (const exception& e)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Failed to write \"" << tobas::kIMUTopic << "\": " << e.what());
  }
}

void ROSBagRecorderNode::magCb(const tobas_msgs::MagneticField::ConstSharedPtr& msg)
{
  tobas_msgs::MagneticFieldAdapter::convert_to_ros_message(*msg, mag_);

  try
  {
    writer_.write(mag_, ns_ + tobas::kMagTopic, get_clock()->now());
  }
  catch (const exception& e)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Failed to write \"" << tobas::kMagTopic << "\": " << e.what());
  }
}

void ROSBagRecorderNode::gnssCb(const tobas_msgs::Gps::ConstSharedPtr& msg)
{
  tobas_msgs::GpsAdapter::convert_to_ros_message(*msg, gps_);

  try
  {
    writer_.write(gps_, ns_ + tobas::kGNSSTopic, get_clock()->now());
  }
  catch (const exception& e)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Failed to write \"" << tobas::kGNSSTopic << "\": " << e.what());
  }
}

void ROSBagRecorderNode::odomCb(const tobas_msgs::Odometry::ConstSharedPtr& msg)
{
  tobas_msgs::OdometryAdapter::convert_to_ros_message(*msg, odom_);

  try
  {
    writer_.write(odom_, ns_ + tobas::kOdometryTopic, get_clock()->now());
  }
  catch (const exception& e)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Failed to write \"" << tobas::kOdometryTopic << "\": " << e.what());
  }
}

void ROSBagRecorderNode::eulerCb(const tobas_kdl_msgs::EulerStamped::ConstSharedPtr& msg)
{
  tobas_kdl_msgs::EulerStampedAdapter::convert_to_ros_message(*msg, euler_);

  try
  {
    writer_.write(euler_, ns_ + tobas::kEulerTopic, get_clock()->now());
  }
  catch (const exception& e)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Failed to write \"" << tobas::kEulerTopic << "\": " << e.what());
  }
}

void ROSBagRecorderNode::windCb(const tobas_msgs::Wind::ConstSharedPtr& msg)
{
  tobas_msgs::WindAdapter::convert_to_ros_message(*msg, wind_);

  try
  {
    writer_.write(wind_, ns_ + tobas::kWindTopic, get_clock()->now());
  }
  catch (const exception& e)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Failed to write \"" << tobas::kWindTopic << "\": " << e.what());
  }
}

void ROSBagRecorderNode::posVelAccYawCmdCb(const tobas_msgs::PosVelAccYaw::ConstSharedPtr& msg)
{
  tobas_msgs::PosVelAccYawAdapter::convert_to_ros_message(*msg, pvay_);

  try
  {
    writer_.write(pvay_, ns_ + tobas::kPosVelAccYawCmdTopic, get_clock()->now());
  }
  catch (const exception& e)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Failed to write \"" << tobas::kPosVelAccYawCmdTopic << "\": " << e.what());
  }
}

void ROSBagRecorderNode::rollPitchYawThrotCmdCb(const tobas_msgs::RollPitchYawThrottle::ConstSharedPtr& msg)
{
  tobas_msgs::RollPitchYawThrottleAdapter::convert_to_ros_message(*msg, rpyt_);

  try
  {
    writer_.write(rpyt_, ns_ + tobas::kRPYThrotCmdTopic, get_clock()->now());
  }
  catch (const exception& e)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Failed to write \"" << tobas::kRPYThrotCmdTopic << "\": " << e.what());
  }
}

void ROSBagRecorderNode::poseTwistAccelCmdCb(const tobas_msgs::PoseTwistAccelCommand::ConstSharedPtr& msg)
{
  tobas_msgs::PoseTwistAccelCommandAdapter::convert_to_ros_message(*msg, pta_);

  try
  {
    writer_.write(pta_, ns_ + tobas::kPoseTwistAccelCmdTopic, get_clock()->now());
  }
  catch (const exception& e)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Failed to write \"" << tobas::kPoseTwistAccelCmdTopic << "\": " << e.what());
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

  // rosbagディレクトリ全体のサイズが大きすぎないか確認
  const auto par_dir_size = path::computeDirectorySize(rosbag_dir_);
  if (par_dir_size > kMaxParDirSize)
  {
    res->success = false;
    res->message = "The size of rosbag directory (" + rosbag_dir_.string() + ") is over "
                   + to_string(kMaxParDirSize / BILLION) + " GB. Please clean it first.";
    return;
  }

  rosbag_path_ = rosbag_dir_ / req->name;
  if (fs::exists(rosbag_path_))
  {
    if (req->overwrite)
    {
      fs::remove_all(rosbag_path_);
    }
    else
    {
      res->success = false;
      res->message = rosbag_path_.string() + " already exists.";
      return;
    }
  }

  rosbag2_storage::StorageOptions options;
  options.uri = rosbag_path_;
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

  registerStandardMsgSub();
  registerNonStandardMsgSub();
  check_size_timer_->reset();

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

  unregisterSubscriptions();
  check_size_timer_->cancel();

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

void ROSBagRecorderNode::checkSizeTimerCb()
{
  const auto rosbag_size = path::computeDirectorySize(rosbag_path_);
  if (rosbag_size > kMaxROSBagSize)
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

    check_size_timer_->cancel();
    is_recording_ = false;

    TOBAS_WARN(
      "The recording is terminated because the size of rosbag ", rosbag_path_, " exceeded ", kMaxROSBagSize / BILLION,
      "GB.");
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ROSBagRecorderNode)
