#include <tobas_constants/constants.hpp>
#include <tobas_node/node.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_tools/util.hpp>

#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/latency.hpp>
#include <tobas_msgs/msg/post_arm_check.hpp>
#include <tobas_msgs_adapter/magnetic_field.hpp>
#include <tobas_msgs_adapter/vibration_level.hpp>

using namespace std::chrono_literals;

class PostArmCheckerNode : public tobas::BaseNode
{
  static constexpr auto kMainTimerPeriod = 100ms;
  static constexpr double kVibrationLevelThresh = 10.;        // [m/s^2]
  static constexpr double kMagLengthErrorThresh = 0.2;        // [-]
  static constexpr double kMagDeclinationThresh = M_PI / 12;  // [rad]
  static constexpr long kLatencyThresh = 1000;                // [us]

  using self = PostArmCheckerNode;
  using super = tobas::BaseNode;

public:
  explicit PostArmCheckerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;
  tobas_msgs::VibrationLevel::ConstSharedPtr vibe_;
  tobas_msgs::MagneticField::ConstSharedPtr mag_;
  tobas_msgs::msg::Latency::ConstSharedPtr latency_;

  ros2::PublisherPtr<tobas_msgs::msg::PostArmCheck> postarm_check_pub_;

  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::VibrationLevel> vibe_sub_;
  ros2::SubscriberPtr<tobas_msgs::MagneticField> mag_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Latency> ctrl_latency_sub_;

  ros2::TimerPtr main_timer_;

  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void vibrationLevelCb(const tobas_msgs::VibrationLevel::ConstSharedPtr& vibe);
  void magCb(const tobas_msgs::MagneticField::ConstSharedPtr& mag);
  void controlLatencyCb(const tobas_msgs::msg::Latency::ConstSharedPtr& latency);

  void mainTimerCb();
};

PostArmCheckerNode::PostArmCheckerNode(const rclcpp::NodeOptions& options) : super("post_arm_checker", options)
{
  postarm_check_pub_ = createPublisher<tobas_msgs::msg::PostArmCheck>(tobas::kPostArmCheckTopic);

  arming_sub_ = createSubscriber(tobas::kArmingTopic, &self::armingCb, this);
  vibe_sub_ = createSubscriber(tobas::kVibrationLevelTopic, &self::vibrationLevelCb, this);
  mag_sub_ = createSubscriber(tobas::kMagTopic, &self::magCb, this);
  ctrl_latency_sub_ = createSubscriber(tobas::kControlLatencyTopic, &self::controlLatencyCb, this);

  main_timer_ = createTimer(kMainTimerPeriod, &self::mainTimerCb, this);
}

void PostArmCheckerNode::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  // アームされたらインスタンス変数を初期化
  if (!arming_ || (arming_->data && !arming->data)) {
    vibe_.reset();
    mag_.reset();
    latency_.reset();
  }

  arming_ = arming;
}

void PostArmCheckerNode::vibrationLevelCb(const tobas_msgs::VibrationLevel::ConstSharedPtr& vibe)
{
  if (!arming_ || !arming_->data) {
    return;
  }

  vibe_ = vibe;
}

void PostArmCheckerNode::magCb(const tobas_msgs::MagneticField::ConstSharedPtr& mag)
{
  if (!arming_ || !arming_->data) {
    return;
  }

  mag_ = mag;
}

void PostArmCheckerNode::controlLatencyCb(const tobas_msgs::msg::Latency::ConstSharedPtr& latency)
{
  if (!arming_ || !arming_->data) {
    return;
  }

  latency_ = latency;
}

void PostArmCheckerNode::mainTimerCb()
{
  if (!arming_ || !arming_->data) {
    return;
  }

  auto postarm_check = std::make_unique<tobas_msgs::msg::PostArmCheck>();

  postarm_check->header.stamp = get_clock()->now();

  if (vibe_) {
    // Check vibration level
    // Vibration levels below 30m/s/s are normally acceptable
    // cf. https://ardupilot.org/copter/docs/common-diagnosing-problems-using-logs.html#vibrations
    postarm_check->vibration_too_high = vibe_->data.max() > kVibrationLevelThresh;
  }
  else {
    TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Vibration level is not received yet.");
    postarm_check->vibration_too_high = true;
  }

  if (mag_) {
    // 地磁気が原点を中心とする単位球上に存在するか
    postarm_check->mag_offset_too_large = (abs(mag_->mag.norm() - 1.) > kMagLengthErrorThresh);

    // 世界座標系から見た磁気ベクトルが参照と一致するか
    // TODO: ESKFから参照地磁気ベクトルを発行して評価
  }
  else {
    TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Magnetic field is not received yet.");
    postarm_check->mag_offset_too_large = true;
    postarm_check->mag_misalignment = true;
  }

  // 制御レイテンシ
  if (latency_) {
    const auto latency_us = ros2::microseconds(latency_->data);
    postarm_check->latency_too_large = (latency_us > kLatencyThresh);
  }
  else {
    TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Control latency is not received yet.");
    postarm_check->latency_too_large = true;
  }

  postarm_check_pub_->publish(move(postarm_check));
}

RCLCPP_COMPONENTS_REGISTER_NODE(PostArmCheckerNode)
