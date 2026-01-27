#include <tobas_constants/constants.hpp>
#include <tobas_dsp/low_pass_filter_p1.hpp>
#include <tobas_kdl/tree_mass_holder.hpp>
#include <tobas_node/node.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/universal_constants.hpp>

#include <tobas_kdl_msgs_adapter/tree.hpp>
#include <tobas_kdl_msgs_adapter/wrench_stamped.hpp>
#include <tobas_msgs/msg/landed_state.hpp>
#include <tobas_msgs_adapter/odometry.hpp>

using namespace std::chrono_literals;

class LandingDetectorNode : public tobas::BaseNode
{
  using self = LandingDetectorNode;
  using super = tobas::BaseNode;

  static constexpr auto kPublishPeriod = 1s;
  static constexpr double kAccelLpfCutoff = 1.;         // [Hz]
  static constexpr double kDistForceLpfCutoff = 1.;     // [Hz]
  static constexpr double kMoveAccelUpThresh = 1.;      // [m/s^2]
  static constexpr double kLandWeightRateThresh = 0.7;  // [-]
  static constexpr auto kTakeoffDetectTimeThresh = 200ms;
  static constexpr auto kLandDetectTimeThresh = 1s;

public:
  explicit LandingDetectorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  bool landed_ = true;
  tobas_msgs::Odometry::ConstSharedPtr odom_;
  tobas_kdl_msgs::WrenchStamped::ConstSharedPtr dist_force_;
  rclcpp::Time t_last_no_change_;  // 最後に鉛直上方向の力が閾値を超えた時刻
  dsp::LowPassFilterP1<double> acc_z_lpf_, force_z_lpf_;

  kdl::Tree tree_;
  kdl::TreeMassHolder mass_holder_;

  ros2::PublisherPtr<tobas_msgs::msg::LandedState> landed_pub_;

  ros2::SubscriberPtr<kdl::Tree> tree_sub_;
  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  ros2::SubscriberPtr<tobas_kdl_msgs::WrenchStamped> dist_force_sub_;

  ros2::TimerPtr publish_timer_;

  void publishCurrentLandedState(const builtin_interfaces::msg::Time& stamp);

  void treeCb(const kdl::Tree::ConstSharedPtr& tree);
  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void disturbanceForceCb(const tobas_kdl_msgs::WrenchStamped::ConstSharedPtr& dist_force);

  void publishTimerCb();
};

LandingDetectorNode::LandingDetectorNode(const rclcpp::NodeOptions& options)
  : super("landing_detector", options), mass_holder_(tree_)
{
  acc_z_lpf_.setCutoffFrequency(kAccelLpfCutoff);
  force_z_lpf_.setCutoffFrequency(kDistForceLpfCutoff);

  landed_pub_ = createPublisher<tobas_msgs::msg::LandedState>(tobas::kLandedTopic);

  tree_sub_ = createSubscriber(tobas::kKdlTreeTopic, &self::treeCb, this, true, true);
  odom_sub_ = createSubscriber(tobas::kOdometryTopic, &self::odomCb, this);
  dist_force_sub_ = createSubscriber(tobas::kDisturbanceForceTopic, &self::disturbanceForceCb, this);

  publish_timer_ = createTimer(kPublishPeriod, &self::publishTimerCb, this);
}

void LandingDetectorNode::publishCurrentLandedState(const builtin_interfaces::msg::Time& stamp)
{
  auto msg = std::make_unique<tobas_msgs::msg::LandedState>();
  msg->header.stamp = stamp;
  msg->data = landed_;
  landed_pub_->publish(std::move(msg));
}

void LandingDetectorNode::treeCb(const kdl::Tree::ConstSharedPtr& tree)
{
  tree_ = *tree;
  mass_holder_.updateInternalDataStructures();
}

void LandingDetectorNode::odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom)
{
  // Get the current time
  const auto& cur_time = odom->header.stamp;

  // Compute the latest vertical accel wrt. the world frame
  const auto acc_W = odom->frame.M * odom->accel.linear;
  const auto& acc_z = acc_W.z();

  // First message
  if (!odom_) {
    odom_ = odom;
    t_last_no_change_ = cur_time;
    acc_z_lpf_.setValue(acc_z);
    return;
  }

  // Smooth the vertical accel
  const auto dt = (cur_time - odom_->header.stamp).seconds();
  acc_z_lpf_.update(acc_z, dt);

  // Update the latest message
  odom_ = odom;

  // Verify the other messages are received
  if (tree_.getNrOfSegments() == 0 || !dist_force_) {
    return;
  }

  if (landed_) {
    // 以下の条件が一定時間続いたら着陸状態から離陸状態に移行
    // 1. 鉛直上方向の加速度が閾値以上
    const auto& acc_z_filt = acc_z_lpf_.getValue();
    if (acc_z_filt > kMoveAccelUpThresh) {
      if (cur_time - t_last_no_change_ > kTakeoffDetectTimeThresh) {
        TOBAS_INFO("Takeoff detected.");
        landed_ = false;
        publishCurrentLandedState(cur_time);
        publish_timer_->reset();
      }
    }
    else {
      t_last_no_change_ = cur_time;
    }
  }
  else {
    // 以下の条件が一定時間続いたら離陸状態から着陸状態に移行
    // 1. 鉛直方向の加速度の絶対値が閾値未満
    // 2. 鉛直上方向の外力（地面反力）が離陸重量の8割以上
    const auto& acc_z_filt = acc_z_lpf_.getValue();
    const auto& force_z_filt = force_z_lpf_.getValue();
    const auto weight = mass_holder_.getMass() * tbs::kGravity;
    const auto force_z_thresh = weight * kLandWeightRateThresh;
    if (std::abs(acc_z_filt) < kMoveAccelUpThresh && force_z_filt > force_z_thresh) {
      if (cur_time - t_last_no_change_ > kLandDetectTimeThresh) {
        TOBAS_INFO("Landing detected.");
        landed_ = true;
        publishCurrentLandedState(cur_time);
        publish_timer_->reset();
      }
    }
    else {
      t_last_no_change_ = cur_time;
    }
  }
}

void LandingDetectorNode::disturbanceForceCb(const tobas_kdl_msgs::WrenchStamped::ConstSharedPtr& dist_force)
{
  // Get the latest vertical force wrt. the world frame
  const auto& force_z = dist_force->wrench.force.z();

  // First message
  if (!dist_force_) {
    dist_force_ = dist_force;
    t_last_no_change_ = dist_force->header.stamp;
    force_z_lpf_.setValue(force_z);
    return;
  }

  // Smooth the vertical force
  const auto dt = (dist_force->header.stamp - dist_force_->header.stamp).seconds();
  force_z_lpf_.update(force_z, dt);

  // Update the latest message
  dist_force_ = dist_force;
}

void LandingDetectorNode::publishTimerCb()
{
  publishCurrentLandedState(now());
}

RCLCPP_COMPONENTS_REGISTER_NODE(LandingDetectorNode)
