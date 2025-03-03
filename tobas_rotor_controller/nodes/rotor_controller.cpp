#include <tobas_math/core.hpp>
#include <tobas_algorithm/core.hpp>
#include <tobas_std_tools/vector.hpp>
#include <tobas_std_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>

#include <tobas_msgs/msg/rotor_thrust_array.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>
#include <tobas_msgs/msg/engine_throttle.hpp>
#include <tobas_msgs/msg/propeller_pitch_angle_array.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/pre_arm_check.hpp>
#include <tobas_msgs/srv/set_arm.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>

using namespace std;

/* 推進系の目標推力を実現する． */
class RotorControllerNode : public tobas::BaseNode
{
  using self = RotorControllerNode;
  using super = tobas::BaseNode;

  using SetArm = tobas_msgs::srv::SetArm;

public:
  explicit RotorControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas::Drone::ConstSharedPtr drone_;
  tobas_msgs::msg::PreArmCheck::ConstSharedPtr prearm_check_;

  bool is_armed_ = false;

  ros2::PublisherPtr<tobas_msgs::msg::RotorSpeedArray> rotor_speeds_pub_;
  ros2::PublisherPtr<tobas_msgs::msg::EngineThrottle> engine_throt_pub_;
  ros2::PublisherPtr<tobas_msgs::msg::PropellerPitchAngleArray> pitches_pub_;
  ros2::PublisherPtr<tobas_msgs::msg::Arming> arming_pub_;

  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorThrustArray> tar_thrusts_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::PreArmCheck> prearm_check_sub_;

  ros2::ServiceServerPtr<SetArm> set_arm_ss_;

  ros2::TimerPtr publish_arming_timer_;
  ros2::TimerPtr auto_disarm_timer_;

  void publishArming();
  void arm();
  void disarm();

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void thrustsCmdCb(const tobas_msgs::msg::RotorThrustArray::ConstSharedPtr& tar_thrusts_msg);
  void preArmCheckCb(const tobas_msgs::msg::PreArmCheck::ConstSharedPtr& prearm_check);

  void setArmCb(const SetArm::Request::ConstSharedPtr& req, const SetArm::Response::SharedPtr& res);

  void autoDisarmTimerCb();
};

RotorControllerNode::RotorControllerNode(const rclcpp::NodeOptions& options) : super("rotor_controller", options)
{
  rotor_speeds_pub_ = createPublisher<tobas_msgs::msg::RotorSpeedArray>(tobas::kRotorSpeedsCmdTopic);
  engine_throt_pub_ = createPublisher<tobas_msgs::msg::EngineThrottle>(tobas::kEngineThrottleCmdTopic);
  pitches_pub_ = createPublisher<tobas_msgs::msg::PropellerPitchAngleArray>(tobas::kPropellerPitchesCmdTopic);
  arming_pub_ = createPublisher<tobas_msgs::msg::Arming>(tobas::kArmingTopic);

  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
  tar_thrusts_sub_ = createSubscriber(tobas::kRotorThrustsCmdTopic, &self::thrustsCmdCb, this);
  prearm_check_sub_ = createSubscriber(tobas::kPreArmCheckTopic, &self::preArmCheckCb, this);

  set_arm_ss_ = createService<SetArm>(tobas::kSetArmSrv, &self::setArmCb, this);

  publish_arming_timer_ = createTimer(tobas::kPublishArmingPeriod, &self::publishArming, this);
  auto_disarm_timer_ = createTimer(tobas::kAutoDisarmTimeout, &self::autoDisarmTimerCb, this, false);
}

void RotorControllerNode::publishArming()
{
  auto arming_msg = std::make_unique<tobas_msgs::msg::Arming>();
  arming_msg->header.stamp = get_clock()->now();
  arming_msg->data = is_armed_;
  arming_pub_->publish(move(arming_msg));
}

void RotorControllerNode::arm()
{
  is_armed_ = true;
  publishArming();
}

void RotorControllerNode::disarm()
{
  is_armed_ = false;
  publishArming();
}

void RotorControllerNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  drone_ = drone;
}

void RotorControllerNode::thrustsCmdCb(const tobas_msgs::msg::RotorThrustArray::ConstSharedPtr& tar_thrusts_msg)
{
  if (drone_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(
      tobas::kTypicalWarnPeriod, "Command is ignored because drone configuration has not been received yet.");
    return;
  }

  if (!is_armed_)
  {
    TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Command is ignored because the rotors are disarmed.");
    return;
  }

  if (tar_thrusts_msg->thrusts.size() != drone_->prop->numRotors())
  {
    TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Thrust command size mismatch.");
    return;
  }

  switch (drone_->prop->type())
  {
    case tobas::propulsion_system_t::ELECTRIC:
    {
      const auto eprop = boost::polymorphic_pointer_downcast<tobas::ElectricPropulsionSystemConfig>(drone_->prop);

      // Create target speeds message
      auto tar_speeds_msg = std::make_unique<tobas_msgs::msg::RotorSpeedArray>();
      tar_speeds_msg->header = tar_thrusts_msg->header;

      // Convert target thrusts to target speeds
      for (const auto& elem : tar_thrusts_msg->thrusts)
      {
        const auto& link_name = elem.link_name;
        const auto& tar_thrust = elem.thrust;
        const auto erotor = eprop->getRotor(link_name);

        tar_speeds_msg->speeds.emplace_back();
        tar_speeds_msg->speeds.back().link_name = link_name;

        if (tar_thrust >= 0.)
        {
          tar_speeds_msg->speeds.back().speed = erotor->speedFromThrust(tar_thrust);
        }
        else
        {
          TOBAS_WARN_THROTTLE(
            tobas::kTypicalWarnPeriod, "Negative thrust is commanded on rotor \"", link_name, "\": ", tar_thrust,
            " < 0 [N]");
          tar_speeds_msg->speeds.back().speed = 0.;
        }
      }

      // Publish target speeds
      rotor_speeds_pub_->publish(move(tar_speeds_msg));

      break;
    }
    case tobas::propulsion_system_t::ICE:  // 参照ピッチ角を用いて推力を実現する (memo: 3-27)
    {
      const auto iprop = boost::polymorphic_pointer_downcast<tobas::ICEPropulsionSystemConfig>(drone_->prop);

      // 合計トルクとその係数を求める
      double torque_sum = 0.;
      double torque_coef_sum = 0.;
      for (const auto& elem : tar_thrusts_msg->thrusts)
      {
        if (elem.thrust < 0.)
        {
          TOBAS_WARN_THROTTLE(
            tobas::kTypicalWarnPeriod, "Negative thrust is commanded on rotor \"", elem.link_name, "\": ", elem.thrust,
            " < 0 [N]");
          continue;
        }

        const auto irotor = iprop->getRotor(elem.link_name);
        torque_sum += irotor->moment_const * elem.thrust;
        torque_coef_sum += irotor->motorConst(irotor->pitch_ref) * irotor->moment_const / math::sqr(irotor->gear_ratio);
      }

      // エンジン回転数を求める
      const auto engine_speed = sqrt(torque_sum / torque_coef_sum);

      // エンジンスロットルを発行
      auto engine_throt_msg = std::make_unique<tobas_msgs::msg::EngineThrottle>();
      engine_throt_msg->header = tar_thrusts_msg->header;
      engine_throt_msg->data = engine_speed > 0. ? iprop->engine.computeThrottle(torque_sum, engine_speed) : 0.;
      engine_throt_pub_->publish(move(engine_throt_msg));

      // プロペラピッチ角を発行
      auto pitches_msg = std::make_unique<tobas_msgs::msg::PropellerPitchAngleArray>();
      pitches_msg->header = tar_thrusts_msg->header;
      for (const auto& elem : tar_thrusts_msg->thrusts)
      {
        const auto irotor = iprop->getRotor(elem.link_name);
        pitches_msg->angles.emplace_back();
        pitches_msg->angles.back().link_name = elem.link_name;
        pitches_msg->angles.back().angle = engine_speed > 0. ? irotor->pitchFromThrust(engine_speed, elem.thrust) : 0.;
      }
      pitches_pub_->publish(move(pitches_msg));

      break;
    }
    default:
    {
      TOBAS_ERROR("Invalid propulsion system type: ", (int)drone_->prop->type());
      break;
    }
  }

  // Reset timeout timers
  auto_disarm_timer_->reset();
}

void RotorControllerNode::preArmCheckCb(const tobas_msgs::msg::PreArmCheck::ConstSharedPtr& prearm_check)
{
  prearm_check_ = prearm_check;
}

void RotorControllerNode::setArmCb(const SetArm::Request::ConstSharedPtr& req, const SetArm::Response::SharedPtr& res)
{
  if (!is_armed_ && req->arming)
  {
    if (prearm_check_ == nullptr)
    {
      res->success = false;
      res->message = "Pre-arm check status is not received yet.";
      return;
    }

    if (!prearm_check_->ok)
    {
      res->success = false;
      res->message = "Pre-arm check failed.";
      return;
    }

    arm();
    auto_disarm_timer_->reset();
  }
  else if (is_armed_ && !req->arming)
  {
    disarm();
    auto_disarm_timer_->cancel();
  }

  res->success = true;
}

void RotorControllerNode::autoDisarmTimerCb()
{
  disarm();
  auto_disarm_timer_->cancel();

  TOBAS_WARN(
    "All rotors are automatically disarmed because ", tobas::kAutoDisarmTimeout.count(),
    " s have elapsed since the last command.");
}

RCLCPP_COMPONENTS_REGISTER_NODE(RotorControllerNode)
