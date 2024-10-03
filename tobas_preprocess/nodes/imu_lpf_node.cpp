#include <tobas_dsp/low_pass_filter.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs_adapter/Imu.hpp>

using namespace std;

class IMULPFNode : public tobas::BaseNode
{
  static constexpr double kGyroLpfCutoff = 40.;   // [Hz] Same as the default IMU_GYRO_CUTOFF (PX4)
  static constexpr double kAccelLpfCutoff = 30.;  // [Hz] Same as the default IMU_ACCEL_CUTOFF (PX4)

  using self = IMULPFNode;
  using super = tobas::BaseNode;

public:
  explicit IMULPFNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  dsp::LowPassFilter<kdl::Vector> gyro_lpf_, accel_lpf_;
  tobas_msgs::Imu::ConstSharedPtr last_msg_;

  ros2::PublisherPtr<tobas_msgs::Imu> imu_lpf_pub_;
  ros2::SubscriberPtr<tobas_msgs::Imu> imu_raw_sub_;

  void imuRawCb(const tobas_msgs::Imu::ConstSharedPtr& imu_raw);
};

IMULPFNode::IMULPFNode(const rclcpp::NodeOptions& options) : super("imu_lpf", options)
{
  imu_lpf_pub_ = createPublisher<tobas_msgs::Imu>(tobas::kImuLpfTopic);
  imu_raw_sub_ = createSubscriber(tobas::kImuTopic, &self::imuRawCb, this);
}

void IMULPFNode::imuRawCb(const tobas_msgs::Imu::ConstSharedPtr& imu_raw)
{
  if (last_msg_ == nullptr)
  {
    TOBAS_INFO("First raw IMU message is received.");
    gyro_lpf_.initialize(kGyroLpfCutoff, imu_raw->gyro);
    accel_lpf_.initialize(kAccelLpfCutoff, imu_raw->accel);
    last_msg_ = imu_raw;
    return;
  }

  const auto dt = (imu_raw->header.stamp - last_msg_->header.stamp).seconds();
  last_msg_ = imu_raw;

  if (gyro_lpf_.update(imu_raw->gyro, dt) < 0)
    TOBAS_ERROR("Failed to update gyro LPF: ", gyro_lpf_.errorMessage(), " dt = ", dt);
  if (accel_lpf_.update(imu_raw->accel, dt) < 0)
    TOBAS_ERROR("Failed to update accel LPF: ", accel_lpf_.errorMessage(), " dt = ", dt);

  auto imu_filtered = std::make_unique<tobas_msgs::Imu>(*imu_raw);
  imu_filtered->gyro = gyro_lpf_.getOutput();
  imu_filtered->accel = accel_lpf_.getOutput();
  imu_lpf_pub_->publish(move(imu_filtered));
}

RCLCPP_COMPONENTS_REGISTER_NODE(IMULPFNode)
