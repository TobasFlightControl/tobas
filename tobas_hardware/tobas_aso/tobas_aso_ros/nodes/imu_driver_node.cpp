#include <tobas_real_common/constants.hpp>
#include <tobas_hardware_common/base_sensor_node.hpp>
#include <tobas_msgs_adapter/ImuStamped.hpp>

#include <tobas_aso_core/ism330dlc.hpp>

using namespace std;

class IMUDriverNode : public hardware::BaseSensorNode
{
  static constexpr auto kSamplingPeriod = 2500us;  // 400Hz

  using self = IMUDriverNode;
  using super = hardware::BaseSensorNode;

public:
  explicit IMUDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  aso::ISM330DLC imu_;
  ros2::PublisherPtr<tobas_msgs::ImuStamped> imu_pub_;

  void mainTimerCb();
};

IMUDriverNode::IMUDriverNode(const rclcpp::NodeOptions& options) : super("aso_imu_driver", options)
{
  if (!imu_.initialize())
    TOBAS_EXIT("Failed to initialize IMU.");

  imu_pub_ = createPublisher<tobas_msgs::ImuStamped>(real::kIMUTopic);
  main_timer_ = createTimer(kSamplingPeriod, &self::mainTimerCb, this);
}

void IMUDriverNode::mainTimerCb()
{
  // Create messages
  auto msg = std::make_unique<tobas_msgs::ImuStamped>();

  // Fill headers
  msg->header.stamp = get_clock()->now();

  // Read IMU
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

  // Publish message
  imu_pub_->publish(move(msg));
}

RCLCPP_COMPONENTS_REGISTER_NODE(IMUDriverNode)
