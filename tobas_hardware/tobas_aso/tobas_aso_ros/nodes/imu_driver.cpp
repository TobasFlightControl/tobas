#include <tobas_tools/imu_sampling_time_publisher.hpp>
#include <tobas_real_common/constants.hpp>
#include <tobas_hardware_common/base_sensor_node.hpp>
#include <tobas_msgs_adapter/imu_stamped.hpp>

#include <tobas_aso_core/ism330dlc.hpp>

#include "./common.hpp"

using namespace std;

class ImuDriverNode : public hardware::BaseSensorNode
{
  static constexpr auto kSamplingPeriod = 1250us;  // 800Hz

  using self = ImuDriverNode;
  using super = hardware::BaseSensorNode;

public:
  explicit ImuDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  aso::ISM330DLC imu_;
  ros2::PublisherPtr<tobas_msgs::ImuStamped> imu_pub_;
  tobas::ImuSamplingTimePublisher sampling_time_pub_;
  ros2::TimerPtr initialize_timer_;

  void initialize();
  void mainTimerCb();
};

ImuDriverNode::ImuDriverNode(const rclcpp::NodeOptions& options) : super("aso_imu_driver", options)
{
  initialize_timer_ = createTimer(aso::kRetryInitializationInterval, &self::initialize, this);
}

void ImuDriverNode::initialize()
{
  if (!imu_.initialize())
  {
    TOBAS_ERROR("Failed to initialize IMU. Retrying...");
    return;
  }

  imu_pub_ = createPublisher<tobas_msgs::ImuStamped>(real::kImuTopic);
  sampling_time_pub_.initialize(shared_from_this(), get_clock()->now());

  initialize_timer_.reset();
  main_timer_ = createTimer(kSamplingPeriod, &self::mainTimerCb, this);
}

void ImuDriverNode::mainTimerCb()
{
  // Get current time
  const auto now = get_clock()->now();

  // Create IMU message
  auto msg = std::make_unique<tobas_msgs::ImuStamped>();
  msg->header.stamp = now;

  // Read IMU data
  if (!imu_.readAcc(msg->imu.accel.x(), msg->imu.accel.y(), msg->imu.accel.z()))
  {
    TOBAS_FATAL("Failed to read accelerometer.");
    return;
  }
  if (!imu_.readGyro(msg->imu.gyro.x(), msg->imu.gyro.y(), msg->imu.gyro.z()))
  {
    TOBAS_FATAL("Failed to read gyroscope.");
    return;
  }

  // Publish IMU message
  imu_pub_->publish(move(msg));

  // Publish sampling time
  sampling_time_pub_.publish(now);
}

RCLCPP_COMPONENTS_REGISTER_NODE(ImuDriverNode)
