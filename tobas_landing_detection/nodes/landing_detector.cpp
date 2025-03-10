#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_kdl/tree_mass_holder.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_std_msgs/msg/bool_stamped.hpp>
#include <tobas_kdl_msgs_adapter/tree.hpp>
#include <tobas_kdl_msgs_adapter/wrench_stamped.hpp>

using namespace std;

class LandingDetectorNode : public tobas::BaseNode
{
  using self = LandingDetectorNode;
  using super = tobas::BaseNode;

  static constexpr auto kPublishPeriod = 1s;

public:
  explicit LandingDetectorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Dynamic parameters
  double switch_time_thresh_;  // 着陸検知がどれだけの時間続いたら状態を切り替えるか [s]
  double switch_mass_rate_;    // 質量の何割の力を検知したら着陸しているとみなすか [-]

  bool landed_ = true;
  rclcpp::Time t_last_detect_;  // 最後に鉛直上方向の力が閾値を超えた時刻

  kdl::Tree tree_;
  kdl::TreeMassHolder mass_holder_;

  ros2::PublisherPtr<tobas_std_msgs::msg::BoolStamped> landed_pub_;

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
  addDynamicDoubleParam("switch_time_threshold", &self::switchTimeThreshCb, this, 0., 1., 5.);
  addDynamicIntParam("switch_mass_rate", &self::switchMassRateCb, this, 20, 50, 80);

  landed_pub_ = createPublisher<tobas_std_msgs::msg::BoolStamped>(tobas::kLandedTopic);

  tree_sub_ = createSubscriber(tobas::kKDLTreeTopic, &self::treeCb, this, true, true);
  dist_force_sub_ = createSubscriber(tobas::kDisturbanceForceTopic, &self::disturbanceForceCb, this);

  publish_timer_ = createTimer(kPublishPeriod, &self::publishTimerCb, this);
}

void LandingDetectorNode::publishLandedState(const builtin_interfaces::msg::Time& stamp)
{
  auto msg = std::make_unique<tobas_std_msgs::msg::BoolStamped>();
  msg->header.stamp = stamp;
  msg->data = landed_;
  landed_pub_->publish(move(msg));
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
  if (mass_holder_.getMass() == 0.)
    return;

  if (t_last_detect_.nanoseconds() == 0)
  {
    t_last_detect_ = dist_force->header.stamp;
    return;
  }

  const auto& z_force = dist_force->wrench.force.z();
  const auto& z_forcd_thresh = mass_holder_.getMass() * tobas_std::kGravity * switch_mass_rate_;

  // 鉛直上方向の力が閾値を超えている状態が一定時間続いたら状態を切り替える
  if ((landed_ && z_force > z_forcd_thresh) || (!landed_ && z_force < z_forcd_thresh))
  {
    t_last_detect_ = dist_force->header.stamp;
  }
  else
  {
    const auto time_from_last_detect = (dist_force->header.stamp - t_last_detect_).seconds();
    if (time_from_last_detect > switch_time_thresh_)
    {
      landed_ = !landed_;
      publishLandedState(dist_force->header.stamp);
    }
  }
}

void LandingDetectorNode::publishTimerCb()
{
  publishLandedState(get_clock()->now());
}

RCLCPP_COMPONENTS_REGISTER_NODE(LandingDetectorNode)
