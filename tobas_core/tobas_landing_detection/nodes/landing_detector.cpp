#include <tobas_constants/constants.hpp>
#include <tobas_dsp/low_pass_filter_p1.hpp>
#include <tobas_kdl/tree_mass_holder.hpp>
#include <tobas_node/node.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/universal_constants.hpp>

#include <tobas_kdl_msgs_adapter/tree.hpp>
#include <tobas_kdl_msgs_adapter/wrench_stamped.hpp>
#include <tobas_msgs/msg/landed_state.hpp>

using namespace std::chrono_literals;

class LandingDetectorNode : public tobas::BaseNode
{
  using self = LandingDetectorNode;
  using super = tobas::BaseNode;

  static constexpr auto kPublishPeriod = 1s;
  static constexpr double kDistForceLpfCutoff = 1.;  // [Hz]

public:
  explicit LandingDetectorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Dynamic parameters
  double switch_time_thresh_;  // 着陸検知がどれだけの時間続いたら状態を切り替えるか [s]
  double switch_mass_rate_;    // 質量の何割の力を検知したら着陸しているとみなすか [-]

  bool landed_ = true;
  tobas_kdl_msgs::WrenchStamped::ConstSharedPtr dist_force_;
  rclcpp::Time t_last_detect_;  // 最後に鉛直上方向の力が閾値を超えた時刻
  dsp::LowPassFilterP1<double> z_force_lpf_;

  kdl::Tree tree_;
  kdl::TreeMassHolder mass_holder_;

  ros2::PublisherPtr<tobas_msgs::msg::LandedState> landed_pub_;

  ros2::SubscriberPtr<kdl::Tree> tree_sub_;
  ros2::SubscriberPtr<tobas_kdl_msgs::WrenchStamped> dist_force_sub_;

  ros2::TimerPtr publish_timer_;

  void publishLandedState(const builtin_interfaces::msg::Time& stamp);

  bool switchTimeThreshCb(const double& p);
  bool switchMassRateCb(const long& p);

  void treeCb(const kdl::Tree::ConstSharedPtr& tree);
  void disturbanceForceCb(const tobas_kdl_msgs::WrenchStamped::ConstSharedPtr& dist_force);

  void publishTimerCb();
};

LandingDetectorNode::LandingDetectorNode(const rclcpp::NodeOptions& options)
  : super("landing_detector", options), mass_holder_(tree_)
{
  z_force_lpf_.setCutoffFrequency(kDistForceLpfCutoff);

  addDynamicDoubleParam("switch_time_threshold", &self::switchTimeThreshCb, this, 0.5, 2, 0, 10, " s");
  addDynamicIntParam("switch_mass_rate", &self::switchMassRateCb, this, 30, 1, 99, " %");  // TODO: 誤着陸を防ぐべき

  landed_pub_ = createPublisher<tobas_msgs::msg::LandedState>(tobas::kLandedTopic);

  tree_sub_ = createSubscriber(tobas::kKdlTreeTopic, &self::treeCb, this, true, true);
  dist_force_sub_ = createSubscriber(tobas::kDisturbanceForceTopic, &self::disturbanceForceCb, this);

  publish_timer_ = createTimer(kPublishPeriod, &self::publishTimerCb, this);
}

void LandingDetectorNode::publishLandedState(const builtin_interfaces::msg::Time& stamp)
{
  auto msg = std::make_unique<tobas_msgs::msg::LandedState>();
  msg->header.stamp = stamp;
  msg->data = landed_;
  landed_pub_->publish(std::move(msg));
}

bool LandingDetectorNode::switchTimeThreshCb(const double& p)
{
  switch_time_thresh_ = p;
  return true;
}

bool LandingDetectorNode::switchMassRateCb(const long& p)
{
  switch_mass_rate_ = static_cast<double>(p) / 100.;
  return true;
}

void LandingDetectorNode::treeCb(const kdl::Tree::ConstSharedPtr& tree)
{
  tree_ = *tree;
  mass_holder_.updateInternalDataStructures();
}

void LandingDetectorNode::disturbanceForceCb(const tobas_kdl_msgs::WrenchStamped::ConstSharedPtr& dist_force)
{
  // Verify the KDL tree is received
  if (mass_holder_.getMass() == 0.) {
    return;
  }

  // Get the latest vertical force
  const auto& z_force = dist_force->wrench.force.z();

  // First message
  if (!dist_force_) {
    dist_force_ = dist_force;
    t_last_detect_ = dist_force->header.stamp;
    z_force_lpf_.setValue(z_force);
    return;
  }

  // Smooth vertical force
  const auto dt = (dist_force->header.stamp - dist_force_->header.stamp).seconds();
  z_force_lpf_.update(z_force, dt);
  const auto& z_force_filtered = z_force_lpf_.getValue();

  // Compute the vertical force threshold
  const auto z_force_thresh = mass_holder_.getMass() * tobas_std::kGravity * switch_mass_rate_;

  // 鉛直上方向の力が閾値を超えている状態が一定時間続いたら状態を切り替える
  if ((landed_ && z_force_filtered > z_force_thresh) || (!landed_ && z_force_filtered < z_force_thresh)) {
    t_last_detect_ = dist_force->header.stamp;
  }
  else {
    const auto time_from_last_detect = (dist_force->header.stamp - t_last_detect_).seconds();
    if (time_from_last_detect > switch_time_thresh_) {
      landed_ = !landed_;
      publishLandedState(dist_force->header.stamp);
    }
  }

  // Update the latest message
  dist_force_ = dist_force;
}

void LandingDetectorNode::publishTimerCb()
{
  publishLandedState(now());
}

RCLCPP_COMPONENTS_REGISTER_NODE(LandingDetectorNode)
