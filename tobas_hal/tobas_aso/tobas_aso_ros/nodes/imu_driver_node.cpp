#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_core/base_sensor_node.hpp>
#include <tobas_hal_msgs/Imu.hpp>

#include <tobas_aso_core/ism330dlc.hpp>

using namespace std;

class IMUDriverNode : public hal::BaseSensorNode
{
  static constexpr auto kSamplingPeriod = 1250us;  // 800Hz

  using self = IMUDriverNode;
  using super = hal::BaseSensorNode;

public:
  explicit IMUDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  aso::ISM330DLC imu_;
  ros2::PublisherPtr<tobas_hal_msgs::Imu> imu_pub_;

  void mainTimerCb();
};

IMUDriverNode::IMUDriverNode(const rclcpp::NodeOptions& options) : super("aso_imu_driver", options)
{
  if (!imu_.initialize())
    TOBAS_EXIT("Failed to initialize IMU.");

  imu_pub_ = createPublisher<tobas_hal_msgs::Imu>(hal::kImuTopic);
  main_timer_ = createTimer(kSamplingPeriod, &self::mainTimerCb, this);
}

void IMUDriverNode::mainTimerCb()
{
  // Create messages
  auto msg = std::make_unique<tobas_hal_msgs::Imu>();

  // Fill headers
  msg->header.stamp = get_clock()->now();

  // Read IMU
  if (!imu_.readAcc(msg->accel.x(), msg->accel.y(), msg->accel.z()))
  {
    TOBAS_FATAL("Failed to read accelerometer.");
    return;
  }
  if (!imu_.readGyro(msg->gyro.x(), msg->gyro.y(), msg->gyro.z()))
  {
    TOBAS_FATAL("Failed to read gyroscope.");
    return;
  }

  // TODO: 軸や符号の変換が必要かも

  // Publish message
  imu_pub_->publish(move(msg));
}

RCLCPP_COMPONENTS_REGISTER_NODE(IMUDriverNode)
