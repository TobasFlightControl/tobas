#include <boost/polymorphic_pointer_cast.hpp>

#include <tobas_algorithm/core.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_math/core.hpp>
#include <tobas_node/node.hpp>
#include <tobas_std_tools/vector.hpp>

#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/ice_propulsion_system_command.hpp>
#include <tobas_msgs/msg/pre_arm_check.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>
#include <tobas_msgs/msg/rotor_thrust_array.hpp>
#include <tobas_msgs/srv/set_arm.hpp>

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
  ros2::PublisherPtr<tobas_msgs::msg::IcePropulsionSystemCommand> ice_cmd_pub_;
  ros2::PublisherPtr<tobas_msgs::msg::Arming> arming_pub_;

  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorThrustArray> tar_thrusts_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::PreArmCheck> prearm_check_sub_;

  ros2::ServiceServerPtr<SetArm> set_arm_ss_;

  ros2::TimerPtr publish_arming_timer_;
  ros2::TimerPtr auto_disarm_timer_;

  void publishArming();

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void thrustsCmdCb(const tobas_msgs::msg::RotorThrustArray::ConstSharedPtr& tar_thrusts_msg);
  void preArmCheckCb(const tobas_msgs::msg::PreArmCheck::ConstSharedPtr& prearm_check);

  void setArmCb(const SetArm::Request::ConstSharedPtr& req, const SetArm::Response::SharedPtr& res);

  void publishArmingTimerCb();
  void autoDisarmTimerCb();
};

RotorControllerNode::RotorControllerNode(const rclcpp::NodeOptions& options) : super("rotor_controller", options)
{
  rotor_speeds_pub_ = createPublisher<tobas_msgs::msg::RotorSpeedArray>(tobas::kRotorSpeedsCmdTopic);
  ice_cmd_pub_ = createPublisher<tobas_msgs::msg::IcePropulsionSystemCommand>(tobas::kIcePropulsionSystemCmdTopic);
  arming_pub_ = createPublisher<tobas_msgs::msg::Arming>(tobas::kArmingTopic);

  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
  tar_thrusts_sub_ = createSubscriber(tobas::kRotorThrustsCmdTopic, &self::thrustsCmdCb, this);
  prearm_check_sub_ = createSubscriber(tobas::kPreArmCheckTopic, &self::preArmCheckCb, this);

  set_arm_ss_ = createService<SetArm>(tobas::kSetArmSrv, &self::setArmCb, this);

  publish_arming_timer_ = createTimer(tobas::kPublishArmingPeriod, &self::publishArmingTimerCb, this);
  auto_disarm_timer_ = createTimer(tobas::kAutoDisarmTimeout, &self::autoDisarmTimerCb, this, false);
}

void RotorControllerNode::publishArming()
{
  auto arming_msg = std::make_unique<tobas_msgs::msg::Arming>();
  arming_msg->header.stamp = get_clock()->now();
  arming_msg->data = is_armed_;
  arming_pub_->publish(move(arming_msg));
}

void RotorControllerNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  if (!drone->isValid()) {
    TOBAS_ERROR("Drone configuration is invalid.");
    return;
  }

  drone_ = drone;
}

void RotorControllerNode::thrustsCmdCb(const tobas_msgs::msg::RotorThrustArray::ConstSharedPtr& tar_thrusts_msg)
{
  if (!drone_) {
    return;
  }

  if (!is_armed_) {
    TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Command is ignored because the rotors are disarmed.");
    return;
  }

  if (tar_thrusts_msg->thrusts.size() != drone_->prop->numRotors()) {
    TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Thrust command size mismatch.");
    return;
  }

  switch (drone_->prop->type()) {
    case tobas::PropulsionSystem::kElectric: {
      const auto eprop = boost::polymorphic_pointer_downcast<tobas::ElectricPropulsionSystemConfig>(drone_->prop);

      // Create target speeds message
      auto tar_speeds_msg = std::make_unique<tobas_msgs::msg::RotorSpeedArray>();
      tar_speeds_msg->header = tar_thrusts_msg->header;

      // Convert target thrusts to target speeds
      for (const auto& elem : tar_thrusts_msg->thrusts) {
        const auto erotor = eprop->getRotor(elem.link_name);
        if (!erotor) {
          TOBAS_ERROR("Electric rotor \"" + elem.link_name + "\" does not exist.");
          continue;
        }
        const auto tar_thrust = std::max(elem.thrust, 0.);
        tar_speeds_msg->speeds.emplace_back();
        tar_speeds_msg->speeds.back().link_name = elem.link_name;
        tar_speeds_msg->speeds.back().speed = erotor->speedFromThrust(tar_thrust);
      }

      // Publish target speeds
      rotor_speeds_pub_->publish(move(tar_speeds_msg));

      break;
    }
    case tobas::PropulsionSystem::kIce:  // 参照ピッチ角を用いて推力を実現する (memo: 3-27)
    {
      const auto iprop = boost::polymorphic_pointer_downcast<tobas::IcePropulsionSystemConfig>(drone_->prop);

      // エンジン軸にかかる合計トルクとその係数を求める
      double thrust_sum = 0.;
      double torque_sum = 0.;
      double K = 0.;
      for (const auto& elem : tar_thrusts_msg->thrusts) {
        const auto irotor = iprop->getRotor(elem.link_name);
        if (!irotor) {
          TOBAS_ERROR("ICE Rotor \"" + elem.link_name + "\" does not exist.");
          continue;
        }
        const auto tar_thrust = std::max(elem.thrust, 0.);
        thrust_sum += tar_thrust;
        torque_sum += irotor->moment_const * tar_thrust / irotor->gear_ratio;  // 減速比を考慮
        K += irotor->motorConst(irotor->pitch_ref) * irotor->moment_const / math::cube(irotor->gear_ratio);
      }

      // コマンドを作成
      auto ice_cmd_msg = std::make_unique<tobas_msgs::msg::IcePropulsionSystemCommand>();
      ice_cmd_msg->header = tar_thrusts_msg->header;

      // エンジンスロットルとプロペラピッチ角を決める
      if (thrust_sum <= 0.) {
        ice_cmd_msg->engine_throttle = 0.;
        for (const auto& elem : tar_thrusts_msg->thrusts) {
          const auto irotor = iprop->getRotor(elem.link_name);
          ice_cmd_msg->pitch_angles.emplace_back();
          ice_cmd_msg->pitch_angles.back().link_name = elem.link_name;
          ice_cmd_msg->pitch_angles.back().angle = irotor->pitch_ref;
        }
      }
      else {
        const auto engine_speed = sqrt(torque_sum / K);
        ice_cmd_msg->engine_throttle = iprop->engine.computeThrottle(torque_sum, engine_speed);
        for (const auto& elem : tar_thrusts_msg->thrusts) {
          const auto irotor = iprop->getRotor(elem.link_name);
          const auto tar_thrust = std::max(elem.thrust, 0.);
          ice_cmd_msg->pitch_angles.emplace_back();
          ice_cmd_msg->pitch_angles.back().link_name = elem.link_name;
          ice_cmd_msg->pitch_angles.back().angle = irotor->pitchFromThrust(engine_speed, tar_thrust);
        }
      }

      // コマンドを発行
      ice_cmd_pub_->publish(move(ice_cmd_msg));

      break;
    }
    default: {
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
  if (!is_armed_ && req->arming) {
    if (!prearm_check_) {
      res->success = false;
      res->message = "Pre-arm check status is not received yet.";
      return;
    }

    if (!prearm_check_->ok) {
      res->success = false;
      res->message = "Pre-arm check failed.";
      return;
    }

    is_armed_ = true;
    publishArming();
    auto_disarm_timer_->reset();
  }
  else if (is_armed_ && !req->arming) {
    is_armed_ = false;
    publishArming();
    auto_disarm_timer_->cancel();
  }

  res->success = true;
  res->message.clear();
}

void RotorControllerNode::publishArmingTimerCb()
{
  publishArming();
}

void RotorControllerNode::autoDisarmTimerCb()
{
  is_armed_ = false;
  publishArming();
  auto_disarm_timer_->cancel();

  TOBAS_WARN(
    "All rotors are automatically disarmed because ",
    tobas::kAutoDisarmTimeout,
    " have elapsed since the last command.");
}

RCLCPP_COMPONENTS_REGISTER_NODE(RotorControllerNode)
