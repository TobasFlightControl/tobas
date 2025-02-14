#include <tobas_math/core.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_tools/util.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/post_arm_check.hpp>
#include <tobas_msgs/msg/latency.hpp>
#include <tobas_msgs_adapter/imu_with_covariance_stamped.hpp>
#include <tobas_msgs_adapter/magnetic_field_with_covariance_stamped.hpp>

using namespace std;
using namespace Eigen;

class PostArmCheckerNode : public tobas::BaseNode
{
  static constexpr auto kMainTimerPeriod = 100ms;

  static constexpr double kGyroNoiseStddevThresh = 0.03;      // [rad/s]
  static constexpr double kAccNoiseStddevThresh = 0.3;        // [m/s^2]
  static constexpr double kMagLengthErrorThresh = 0.2;        // [-]
  static constexpr double kMagDeclinationThresh = M_PI / 12;  // [rad]
  static constexpr long kLatencyThresh = 1000;                // [us]

  using self = PostArmCheckerNode;
  using super = tobas::BaseNode;

public:
  explicit PostArmCheckerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;
  tobas_msgs::ImuWithCovarianceStamped::ConstSharedPtr imu_;
  tobas_msgs::MagneticFieldWithCovarianceStamped::ConstSharedPtr mag_;
  tobas_msgs::msg::Latency::ConstSharedPtr latency_;

  ros2::PublisherPtr<tobas_msgs::msg::PostArmCheck> postarm_check_pub_;

  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::ImuWithCovarianceStamped> imu_sub_;
  ros2::SubscriberPtr<tobas_msgs::MagneticFieldWithCovarianceStamped> mag_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Latency> latency_sub_;

  ros2::TimerPtr main_timer_;

  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void imuCb(const tobas_msgs::ImuWithCovarianceStamped::ConstSharedPtr& imu);
  void magCb(const tobas_msgs::MagneticFieldWithCovarianceStamped::ConstSharedPtr& mag);
  void latencyCb(const tobas_msgs::msg::Latency::ConstSharedPtr& latency);

  void mainTimerCb();
};

PostArmCheckerNode::PostArmCheckerNode(const rclcpp::NodeOptions& options) : super("post_arm_checker", options)
{
  postarm_check_pub_ = createPublisher<tobas_msgs::msg::PostArmCheck>(tobas::kPostArmCheckTopic);

  arming_sub_ = createSubscriber(tobas::kArmingTopic, &self::armingCb, this);
  imu_sub_ = createSubscriber(tobas::kImuTopic, &self::imuCb, this);
  mag_sub_ = createSubscriber(tobas::kMagTopic, &self::magCb, this);
  latency_sub_ = createSubscriber(tobas::kLatencyTopic, &self::latencyCb, this);

  main_timer_ = createTimer(kMainTimerPeriod, &self::mainTimerCb, this);
}

void PostArmCheckerNode::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  arming_ = arming;
}

void PostArmCheckerNode::imuCb(const tobas_msgs::ImuWithCovarianceStamped::ConstSharedPtr& imu)
{
  imu_ = imu;
}

void PostArmCheckerNode::magCb(const tobas_msgs::MagneticFieldWithCovarianceStamped::ConstSharedPtr& mag)
{
  mag_ = mag;
}

void PostArmCheckerNode::latencyCb(const tobas_msgs::msg::Latency::ConstSharedPtr& latency)
{
  latency_ = latency;
}

void PostArmCheckerNode::mainTimerCb()
{
  if (arming_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Arming status is not received yet.");
    return;
  }

  if (!arming_->data)
    return;

  auto postarm_check = std::make_unique<tobas_msgs::msg::PostArmCheck>();

  postarm_check->header.stamp = get_clock()->now();

  if (imu_ != nullptr)
  {
    // ジャイロノイズの標準偏差
    const auto gyro_noise_var = imu_->imu.gyro_covariance.diagonal().maxCoeff();
    postarm_check->gyro_noise_too_large = (gyro_noise_var > math::sqr(kGyroNoiseStddevThresh));
    if (postarm_check->gyro_noise_too_large)
    {
      TOBAS_WARN_THROTTLE(
        tobas::kTypicalWarnPeriod, "Gyro noise stddev is too large: ", sqrt(gyro_noise_var), " > ",
        kGyroNoiseStddevThresh, " [m/s^2]");
    }

    // 加速度ノイズの標準偏差
    const auto acc_noise_var = imu_->imu.accel_covariance.diagonal().maxCoeff();
    postarm_check->accel_noise_too_large = (acc_noise_var > math::sqr(kAccNoiseStddevThresh));
    if (postarm_check->accel_noise_too_large)
    {
      TOBAS_WARN_THROTTLE(
        tobas::kTypicalWarnPeriod, "Accel noise stddev is too large: ", sqrt(acc_noise_var), " > ",
        kAccNoiseStddevThresh, " [m/s^2]");
    }
  }
  else
  {
    TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "IMU state is not received yet.");
    postarm_check->gyro_noise_too_large = true;
    postarm_check->accel_noise_too_large = true;
  }

  if (mag_ != nullptr)
  {
    // 地磁気が原点を中心とする単位球上に存在するか
    postarm_check->mag_offset_too_large = (abs(mag_->mag.mag.norm() - 1.) > kMagLengthErrorThresh);
    if (postarm_check->mag_offset_too_large)
    {
      TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "The offset of the geomagnetic vector is too large.");
    }

    // 世界座標系から見た磁気ベクトルが参照と一致するか
    // TODO: ESKFから参照地磁気ベクトルを発行して評価
  }
  else
  {
    TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Magnetic field is not received yet.");
    postarm_check->mag_offset_too_large = true;
    postarm_check->mag_misalignment = true;
  }

  // 制御レイテンシ
  if (latency_ != nullptr)
  {
    const auto sec = static_cast<long>(latency_->data.sec);
    const auto nsec = static_cast<long>(latency_->data.nanosec);
    const auto latency_us = sec * 1'000'000 + nsec / 1'000;
    postarm_check->latency_too_large = (latency_us > kLatencyThresh);
    if (postarm_check->latency_too_large)
    {
      TOBAS_WARN_THROTTLE(
        tobas::kTypicalWarnPeriod, "Control latency is too large: ", latency_us, " > ", kLatencyThresh, " [us]");
    }
  }
  else
  {
    TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Control latency is not received yet.");
    postarm_check->latency_too_large = true;
  }

  postarm_check_pub_->publish(move(postarm_check));
}

RCLCPP_COMPONENTS_REGISTER_NODE(PostArmCheckerNode)
