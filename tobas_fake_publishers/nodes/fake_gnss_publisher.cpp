#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs_adapter/gps.hpp>

using namespace std;

class FakeGNSSPublisherNode : public tobas::BaseNode
{
  static constexpr auto kSamplingPeriod = 200ms;

  static constexpr double kDefaultPosStddev = 3.;   // [m]
  static constexpr double kDefaultVelStddev = 0.3;  // [m/s]

  using self = FakeGNSSPublisherNode;
  using super = tobas::BaseNode;

public:
  explicit FakeGNSSPublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  double pos_stddev_;
  double vel_stddev_;

  ros2::PublisherPtr<tobas_msgs::Gps> gnss_pub_;
  ros2::TimerPtr timer_;

  void timerCb();
};

FakeGNSSPublisherNode::FakeGNSSPublisherNode(const rclcpp::NodeOptions& options)
  : super("fake_battery_publisher", options)
{
  pos_stddev_ = getDoubleParam("position_stddev", kDefaultPosStddev);
  vel_stddev_ = getDoubleParam("velocity_stddev", kDefaultVelStddev);

  gnss_pub_ = createPublisher<tobas_msgs::Gps>(tobas::kGNSSTopic);
  timer_ = createTimer(kSamplingPeriod, &self::timerCb, this);
}

void FakeGNSSPublisherNode::timerCb()
{
  auto gnss_msg = std::make_unique<tobas_msgs::Gps>();
  gnss_msg->header.stamp = get_clock()->now();
  gnss_msg->fix_type = tobas_msgs::msg::Gps::FIX_3D;
  gnss_msg->latitude = 0.;
  gnss_msg->longitude = 0.;
  gnss_msg->altitude = 0.;
  gnss_msg->ground_speed.setZero();
  gnss_msg->position_covariance = Eigen::Vector3d::Constant(pos_stddev_).asDiagonal();
  gnss_msg->velocity_covariance = Eigen::Vector3d::Constant(vel_stddev_).asDiagonal();

  gnss_pub_->publish(move(gnss_msg));
}

RCLCPP_COMPONENTS_REGISTER_NODE(FakeGNSSPublisherNode)
