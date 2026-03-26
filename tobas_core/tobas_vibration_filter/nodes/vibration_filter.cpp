#include <tobas_constants/ros_interface.hpp>
#include <tobas_dsp/high_pass_filter.hpp>
#include <tobas_dsp/low_pass_filter_p1.hpp>
#include <tobas_node/node.hpp>
#include <tobas_ros2_tools/time.hpp>

#include <tobas_msgs_adapter/imu.hpp>
#include <tobas_msgs_adapter/vibration_level.hpp>

class VibrationFilterNode : public tobas::BaseNode
{
  using self = VibrationFilterNode;
  using super = tobas::BaseNode;

  // ArduPilot: fc_hpf = 5Hz, fc_lpf = 2Hz
  // https://firmware.ardupilot.org/coverage/AP_InertialSensor/AP_InertialSensor.h.gcov.html
  static constexpr double kHpfCutoff = 10.;  // [Hz] (G(1Hz) ~ 0.1, G(30Hz) ~ 0.95)
  static constexpr double kLpfCutoff = 1.;   // [Hz]

public:
  explicit VibrationFilterNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas_msgs::Imu::ConstSharedPtr imu_;

  dsp::HighPassFilter<kdl::Vector> hpf_;
  dsp::LowPassFilterP1<kdl::Vector> lpf_;

  ros2::PublisherPtr<tobas_msgs::VibrationLevel> vibe_pub_;
  ros2::SubscriberPtr<tobas_msgs::Imu> imu_sub_;

  void imuCb(const tobas_msgs::Imu::ConstSharedPtr& imu);
};

VibrationFilterNode::VibrationFilterNode(const rclcpp::NodeOptions& options)
  : super("vibration_filter", nodeOptions_Default(options))
{
  TOBAS_ASSERT(hpf_.setCutoffFrequency(kHpfCutoff));
  TOBAS_ASSERT(lpf_.setCutoffFrequency(kLpfCutoff));

  vibe_pub_ = createPublisher<tobas_msgs::VibrationLevel>(tobas::topic::kVibrationLevel);
  imu_sub_ = createSubscriber(tobas::topic::kImuRaw, &self::imuCb, this);
}

void VibrationFilterNode::imuCb(const tobas_msgs::Imu::ConstSharedPtr& imu)
{
  const auto& acc_meas = imu->accel;

  if (!imu_) {
    hpf_.setValue(acc_meas);
    lpf_.setValue(acc_meas);
    imu_ = imu;
    return;
  }

  const auto dt = (imu->header.stamp - imu_->header.stamp).seconds();
  imu_ = imu;

  // 高周波成分の2乗を平滑化することで擬似的にRMSを計算する (ArduPilotのVIBEの計算方法と同じ)
  hpf_.update(acc_meas, dt);
  const auto acc_vibe_sqr = hpf_.getValue().sqr();  // [m^2/s^4]
  lpf_.update(acc_vibe_sqr, dt);
  const auto acc_vibe_rms = lpf_.getValue().sqrt();  // [m/s^2]

  auto vibe = std::make_unique<tobas_msgs::VibrationLevel>();
  vibe->header.stamp = imu->header.stamp;
  vibe->data = acc_vibe_rms;
  vibe_pub_->publish(std::move(vibe));
}

RCLCPP_COMPONENTS_REGISTER_NODE(VibrationFilterNode)
