#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_tools/control_latency_publisher.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_msgs/msg/engine_state.hpp>
#include <tobas_msgs/msg/ice_propulsion_system_command.hpp>
#include <tobas_msgs/msg/pwm_array.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>

using namespace std;

class ICEPropulsionSystemHandlerNode : public tobas::BaseNode
{
  using self = ICEPropulsionSystemHandlerNode;
  using super = tobas::BaseNode;

public:
  explicit ICEPropulsionSystemHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas::Drone::ConstSharedPtr drone_;
  tobas::ICEPropulsionSystemConfig::ConstSharedPtr iprop_;

  map<string, double> pitch_angles_;
  bool is_commanded_ = false;

  ros2::PublisherPtr<tobas_msgs::msg::PwmArray> pwms_pub_;
  ros2::PublisherPtr<tobas_msgs::msg::RotorStateArray> rotor_states_pub_;
  tobas::ControlLatencyPublisher latency_pub_;

  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::EngineState> engine_state_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::IcePropulsionSystemCommand> ice_cmd_sub_;

  ros2::TimerPtr auto_stop_timer_;

  void stopActuator();

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void engineStateCb(const tobas_msgs::msg::EngineState::ConstSharedPtr& engine_state);
  void iceCommandCb(const tobas_msgs::msg::IcePropulsionSystemCommand::ConstSharedPtr& ice_cmd);

  void autoStopTimerCb();
};

ICEPropulsionSystemHandlerNode::ICEPropulsionSystemHandlerNode(const rclcpp::NodeOptions& options)
  : super("real_ice_propulsion_system_handler", options)
{
  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
}

void ICEPropulsionSystemHandlerNode::stopActuator()
{
  // Create command message
  auto pwms = std::make_unique<tobas_msgs::msg::PwmArray>();

  // Engine
  switch (iprop_->engine.hw_iface)
  {
    case tobas::hw_iface_t::PWM:
    {
      const auto& pwm_cfg = drone_->pwms.at(tobas::pwm::kEngineThrottleKey);
      pwms->pwms.emplace_back();
      pwms->pwms.back().channel = pwm_cfg.channel;
      pwms->pwms.back().period = pwm_cfg.periodFromValue(0.);
      break;
    }
    case tobas::hw_iface_t::OTHER:
    {
      break;
    }
    default:
    {
      TOBAS_ERROR("The hardware interface of engine throttle is invalid: ", (int)iprop_->engine.hw_iface);
      break;
    }
  }

  // Pitch angles
  for (const auto& [_, rotor] : iprop_->rotors)
  {
    const auto irotor = boost::polymorphic_pointer_downcast<tobas::ICERotorConfig>(rotor);

    const auto& link_name = irotor->link_name;
    const auto& cmd_angle = irotor->pitch_ref;

    // Set current pitch angle
    pitch_angles_.at(link_name) = cmd_angle;

    // Set command
    switch (irotor->hw_iface)
    {
      case tobas::hw_iface_t::PWM:
      {
        const auto& pwm_cfg = drone_->pwms.at(link_name);
        pwms->pwms.emplace_back();
        pwms->pwms.back().channel = pwm_cfg.channel;
        pwms->pwms.back().period = pwm_cfg.periodFromValue(cmd_angle);
        break;
      }
      case tobas::hw_iface_t::OTHER:
      {
        break;
      }
      default:
      {
        TOBAS_ERROR(
          "The hardware interface of variable pitch propeller \"", link_name, "\" is invalid: ", (int)irotor->hw_iface);
        break;
      }
    }
  }

  // Publish command
  if (pwms->pwms.size() > 0)
  {
    pwms->header.stamp = get_clock()->now();
    pwms_pub_->publish(move(pwms));
  }
}

void ICEPropulsionSystemHandlerNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  if (!drone->prop)
    return;

  if (drone->prop->type() != tobas::propulsion_system_t::ICE)
    return;

  drone_ = drone;
  iprop_ = boost::polymorphic_pointer_downcast<tobas::ICEPropulsionSystemConfig>(drone->prop);

  // Initialize pitch angle map
  pitch_angles_.clear();
  for (const auto& [_, rotor] : iprop_->rotors)
  {
    const auto irotor = boost::polymorphic_pointer_downcast<tobas::ICERotorConfig>(rotor);
    pitch_angles_[irotor->link_name] = irotor->pitch_ref;
  }

  // Register publishers
  pwms_pub_ = createPublisher<tobas_msgs::msg::PwmArray>(tobas::kPwmCmdTopic);
  rotor_states_pub_ = createPublisher<tobas_msgs::msg::RotorStateArray>(tobas::kRotorStatesTopic);
  latency_pub_.initialize(shared_from_this());

  // Register subscribers
  engine_state_sub_ = createSubscriber(tobas::kEngineStateTopic, &self::engineStateCb, this);
  ice_cmd_sub_ = createSubscriber(tobas::kIcePropulsionSystemCmdTopic, &self::iceCommandCb, this);

  // Create timers
  auto_stop_timer_ = createTimer(tobas::kCommandAutoResetTimeout, &self::autoStopTimerCb, this);
}

void ICEPropulsionSystemHandlerNode::engineStateCb(const tobas_msgs::msg::EngineState::ConstSharedPtr& engine_state)
{
  auto rotor_states = std::make_unique<tobas_msgs::msg::RotorStateArray>();
  rotor_states->header.stamp = engine_state->header.stamp;

  for (const auto& [link_name, rotor] : iprop_->rotors)
  {
    const auto irotor = boost::polymorphic_pointer_downcast<tobas::ICERotorConfig>(rotor);

    rotor_states->states.emplace_back();
    rotor_states->states.back().link_name = link_name;
    rotor_states->states.back().speed = irotor->speedEngineToRotor(engine_state->speed);
    rotor_states->states.back().thrust = irotor->thrustFromPitch(engine_state->speed, pitch_angles_.at(link_name));
    rotor_states->states.back().status = tobas_msgs::msg::RotorState::NO_ERROR;
  }

  rotor_states_pub_->publish(move(rotor_states));
}

void ICEPropulsionSystemHandlerNode::iceCommandCb(
  const tobas_msgs::msg::IcePropulsionSystemCommand::ConstSharedPtr& ice_cmd)
{
  // Create command message
  auto pwms = std::make_unique<tobas_msgs::msg::PwmArray>();

  // Engine
  switch (iprop_->engine.hw_iface)
  {
    case tobas::hw_iface_t::PWM:
    {
      const auto& pwm_cfg = drone_->pwms.at(tobas::pwm::kEngineThrottleKey);
      pwms->pwms.emplace_back();
      pwms->pwms.back().channel = pwm_cfg.channel;
      pwms->pwms.back().period = pwm_cfg.periodFromValue(ice_cmd->engine_throttle);
      break;
    }
    case tobas::hw_iface_t::OTHER:
    {
      break;
    }
    default:
    {
      TOBAS_ERROR("The hardware interface of engine throttle is invalid: ", (int)iprop_->engine.hw_iface);
      break;
    }
  }

  // Pitch angles
  for (const auto& elem : ice_cmd->pitch_angles)
  {
    const auto& link_name = elem.link_name;
    auto cmd_angle = elem.angle;

    // Get rotor config
    const auto rotor_it = iprop_->rotors.find(link_name);
    if (rotor_it == iprop_->rotors.end())
    {
      TOBAS_ERROR("Rotor link \"", link_name, "\" is not found.");
      continue;
    }
    const auto irotor = boost::polymorphic_pointer_downcast<tobas::ICERotorConfig>(rotor_it->second);

    // Check pitch angle limit
    if (irotor->pitch_range.inRange(cmd_angle))
    {
      TOBAS_WARN_THROTTLE(
        tobas::kTypicalWarnPeriod, "Commanded pitch angle of propeller \"", link_name,
        "\" is out of range: ", cmd_angle, " ∉ ", irotor->pitch_range);
      cmd_angle = irotor->pitch_range.clamp(cmd_angle);
    }

    // Set current pitch angle
    pitch_angles_.at(link_name) = cmd_angle;

    // Set command
    switch (irotor->hw_iface)
    {
      case tobas::hw_iface_t::PWM:
      {
        const auto& pwm_cfg = drone_->pwms.at(link_name);
        pwms->pwms.emplace_back();
        pwms->pwms.back().channel = pwm_cfg.channel;
        pwms->pwms.back().period = pwm_cfg.periodFromValue(cmd_angle);
        break;
      }
      case tobas::hw_iface_t::OTHER:
      {
        break;
      }
      default:
      {
        TOBAS_ERROR(
          "The hardware interface of variable pitch propeller \"", link_name, "\" is invalid: ", (int)irotor->hw_iface);
        break;
      }
    }
  }

  // Publish command
  if (pwms->pwms.size() > 0)
  {
    pwms->header.stamp = ice_cmd->header.stamp;
    pwms_pub_->publish(move(pwms));
  }

  // Publish control latency
  latency_pub_.publish(ice_cmd->header.stamp);

  // Reset timeout timer
  auto_stop_timer_->reset();

  // Now the propulsion system is commanded
  is_commanded_ = true;
}

void ICEPropulsionSystemHandlerNode::autoStopTimerCb()
{
  stopActuator();

  if (is_commanded_)
  {
    is_commanded_ = false;
    TOBAS_WARN(
      "ICE propulsion system is automatically stopped because ", tobas::kCommandAutoResetTimeout,
      " have elapsed since the last command.");
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ICEPropulsionSystemHandlerNode);
