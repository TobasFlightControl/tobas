#include <std_srvs/srv/trigger.hpp>

#include <tobas_math/core.hpp>
#include <tobas_std_tools/timestamped_buffer.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_ros2_tools/eigen_conversion.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/cpu.hpp>
#include <tobas_msgs_adapter/Odometry.hpp>
#include <tobas_msgs/msg/pre_arm_check.hpp>
#include <tobas_drone_msgs_adapter/Drone.hpp>

using namespace std;
using namespace Eigen;
using namespace std_srvs::srv;

class PreArmCheckerNode : public tobas::BaseNode
{
  static constexpr double kOdomCallbackInterval = 0.1;    // [s]
  static constexpr double kPosDriftCheckTimeWindow = 5.;  // [s]
  static constexpr double kPosDriftThresh = 1.;           // [m]
  static constexpr double kCPUTempThresh = 80.;           // [degC] // TODO: もう少し下げる
  static constexpr double kAttitudeThresh = M_PI / 6;     // [rad/s]
  static constexpr double kHorPosStddevThresh = 1.;       // [m]
  static constexpr double kVerPosStddevThresh = 2.;       // [m]
  static constexpr double kRotStddevThresh = M_PI / 24;   // [rad]
  static constexpr double kVelStddevThresh = 0.3;         // [m/s]
  static constexpr auto kPreArmCheckTimerPeriod = 1s;

  using self = PreArmCheckerNode;
  using super = tobas::BaseNode;

public:
  explicit PreArmCheckerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas::Drone::ConstSharedPtr drone_;
  tobas_msgs::msg::Battery::ConstSharedPtr battery_;
  tobas_msgs::msg::Cpu::ConstSharedPtr cpu_;
  tobas_msgs::Odometry::ConstSharedPtr odom_;

  array<tobas_std::TimestampedBufferDouble, 3> pos_buf_;
  double roll_, pitch_, yaw_;

  ros2::PublisherPtr<tobas_msgs::msg::PreArmCheck> prearm_check_pub_;
  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Battery> batt_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Cpu> cpu_sub_;
  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;

  ros2::TimerPtr main_timer_;

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void battCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery);
  void cpuCb(const tobas_msgs::msg::Cpu::ConstSharedPtr& cpu);
  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);

  void mainTimerCb();
};

PreArmCheckerNode::PreArmCheckerNode(const rclcpp::NodeOptions& options)
  : super("prearm_checker", options),
    pos_buf_{ tobas_std::TimestampedBufferDouble(kPosDriftCheckTimeWindow),
              tobas_std::TimestampedBufferDouble(kPosDriftCheckTimeWindow),
              tobas_std::TimestampedBufferDouble(kPosDriftCheckTimeWindow) }
{
  prearm_check_pub_ = createPublisher<tobas_msgs::msg::PreArmCheck>(tobas::kPreArmCheckTopic, true, true);

  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
  batt_sub_ = createSubscriber(path::join(tobas::kThrottledTopicPrefix, tobas::kBatteryLpfTopic), &self::battCb, this);
  cpu_sub_ = createSubscriber(tobas::kCPUTopic, &self::cpuCb, this);
  odom_sub_ = createSubscriber(tobas::kOdometryTopic, &self::odomCb, this);

  main_timer_ = createTimer(kPreArmCheckTimerPeriod, &self::mainTimerCb, this);
}

void PreArmCheckerNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  drone_ = drone;
}

void PreArmCheckerNode::battCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery)
{
  battery_ = battery;
}

void PreArmCheckerNode::cpuCb(const tobas_msgs::msg::Cpu::ConstSharedPtr& cpu)
{
  cpu_ = cpu;
}

void PreArmCheckerNode::odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom)
{
  if (odom_ == nullptr)
  {
    odom_ = odom;
    return;
  }

  // 評価時の計算量を抑えるために処理頻度を制限
  if ((odom->header.stamp - odom_->header.stamp).seconds() < kOdomCallbackInterval)
    return;

  odom_ = odom;

  const auto stamp = ros2::chronoFromRosTime(odom->header.stamp);
  for (size_t i = 0; i < 3; ++i)
    pos_buf_[i].add(stamp, odom->frame.p(i));
}

void PreArmCheckerNode::mainTimerCb()
{
  if (drone_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Drone configuration is not received yet.");
    return;
  }
  if (battery_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Battery information is not received yet.");
    return;
  }
  if (cpu_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "CPU information is not received yet.");
    return;
  }
  if (odom_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Odometry is not received yet.");
    return;
  }

  auto prearm_check = std::make_unique<tobas_msgs::msg::PreArmCheck>();

  prearm_check->header.stamp = get_clock()->now();
  prearm_check->ok = true;

  // バッテリー電圧
  prearm_check->battery_voltage_too_low = (battery_->voltage < drone_->battery.sag_voltage);
  if (prearm_check->battery_voltage_too_low)
    prearm_check->ok = false;

  // CPU温度
  prearm_check->cpu_temperature_too_high = (cpu_->temperature > kCPUTempThresh);
  if (prearm_check->cpu_temperature_too_high)
    prearm_check->ok = false;

  // 姿勢角
  odom_->frame.M.getRPY(roll_, pitch_, yaw_);
  prearm_check->attitude_too_steep = (max(abs(roll_), abs(pitch_)) > kAttitudeThresh);
  if (prearm_check->attitude_too_steep)
    prearm_check->ok = false;

  // 位置のドリフト
  prearm_check->position_unstable = false;
  for (size_t i = 0; i < pos_buf_.size(); ++i)
  {
    if (!pos_buf_[i].isFilled() || pos_buf_[i].range() > kPosDriftThresh)
    {
      prearm_check->position_unstable = true;
      prearm_check->ok = false;
      break;
    }
  }

  // 位置推定の共分散
  const Vector3d pos_cov_diag = odom_->position_covariance.diagonal();
  const auto hor_pos_var = max(pos_cov_diag.x(), pos_cov_diag.y());
  const auto ver_pos_var = pos_cov_diag.z();
  prearm_check->position_inaccurate =
    (hor_pos_var > math::sqr(kHorPosStddevThresh) || ver_pos_var > math::sqr(kVerPosStddevThresh));
  if (prearm_check->position_inaccurate)
    prearm_check->ok = false;

  // 姿勢推定の共分散
  const auto rot_var = odom_->orientation_covariance.diagonal().maxCoeff();
  prearm_check->orientation_inaccurate = (rot_var > math::sqr(kRotStddevThresh));
  if (prearm_check->orientation_inaccurate)
    prearm_check->ok = false;

  // 速度推定の共分散
  const auto vel_var = odom_->velocity_covariance.diagonal().maxCoeff();
  prearm_check->velocity_inaccurate = (vel_var > math::sqr(kVelStddevThresh));
  if (prearm_check->velocity_inaccurate)
    prearm_check->ok = false;

  prearm_check_pub_->publish(move(prearm_check));
}

RCLCPP_COMPONENTS_REGISTER_NODE(PreArmCheckerNode)
