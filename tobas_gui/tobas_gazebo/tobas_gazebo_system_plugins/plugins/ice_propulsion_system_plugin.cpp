// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <optional>

#include <tobas_constants/ros_interface.hpp>
#include <tobas_constants/throttle.hpp>
#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_conversions/gazebo_kdl.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/time.hpp>

#include <tobas_gazebo_msgs/msg/engine_state.hpp>
#include <tobas_gazebo_msgs/msg/rotor_state.hpp>
#include <tobas_gazebo_msgs/msg/throttle.hpp>
#include <tobas_msgs/msg/engine_state.hpp>
#include <tobas_msgs/msg/ice_propulsion_system_command.hpp>
#include <tobas_msgs/msg/latency.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_msgs_adapter/wind.hpp>

#include "tobas_gazebo_system_plugins/common/common.hpp"
#include "tobas_gazebo_system_plugins/engine_model.hpp"
#include "tobas_gazebo_system_plugins/ice_rotor_model.hpp"
#include "tobas_gazebo_system_plugins/rate_manager.hpp"
#include "tobas_gazebo_system_plugins/sdf.hpp"

namespace ch = std::chrono;

namespace tobas
{
namespace gazebo
{
/* Simulates engine and propellers. */
class GazeboIcePropulsionSystemPlugin : public BaseNode,
                                        public gz::sim::System,
                                        public gz::sim::ISystemConfigure,
                                        public gz::sim::ISystemPreUpdate,
                                        public gz::sim::ISystemPostUpdate
{
  // Constants
  using self = GazeboIcePropulsionSystemPlugin;

public:
  explicit GazeboIcePropulsionSystemPlugin();

  void Configure(
    const gz::sim::Entity& model_entity,
    const sdf::ElementConstPtr& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager&) override;

  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override;
  void PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) override;

private:
  IceRotorModelMap rotors_;
  EngineModel engine_;

  // SDF parameters
  size_t publish_state_rate_;  // [Hz]

  gz::math::Vector3d wind_vel_W_ = gz::math::Vector3d::Zero;  // [m/s]
  builtin_interfaces::msg::Time prev_sim_time_;
  builtin_interfaces::msg::Time last_cmd_time_;  // Time when the last throttle command was issued
  std::optional<RateManager> publish_state_rate_manager_;

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::Latency> latency_pub_;
  ros2::PublisherPtr<tobas_msgs::msg::EngineState> engine_state_pub_;
  ros2::PublisherPtr<tobas_gazebo_msgs::msg::EngineState> engine_state_gt_pub_;
  std::map<std::string, ros2::PublisherPtr<tobas_msgs::msg::RotorState>> rotor_state_pubs_;
  std::map<std::string, ros2::PublisherPtr<tobas_gazebo_msgs::msg::RotorState>> rotor_state_gt_pubs_;

  // Subscribers
  ros2::SubscriberPtr<tobas_msgs::msg::IcePropulsionSystemCommand> ice_cmd_sub_;
  ros2::SubscriberPtr<tobas_msgs::Wind> wind_gt_sub_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
  void registerPubSub();

  void iceCommandCb(const tobas_msgs::msg::IcePropulsionSystemCommand::ConstSharedPtr& ice_cmd);
  void windSpeedGtCb(const tobas_msgs::Wind::ConstSharedPtr& wind_gt);
};

GazeboIcePropulsionSystemPlugin::GazeboIcePropulsionSystemPlugin() : engine_(rotors_)
{
}

void GazeboIcePropulsionSystemPlugin::Configure(
  const gz::sim::Entity& model_entity,
  const sdf::ElementConstPtr& sdf,
  gz::sim::EntityComponentManager& ecm,
  gz::sim::EventManager&)
{
  constexpr char kRotorKey[] = "rotor";
  constexpr char kEngineKey[] = "engine";

  initialize("gazebo_ice_propulsion_system_plugin", sdf);
  getSdfParams(sdf);

  publish_state_rate_manager_.emplace(publish_state_rate_);

  // Get robot model.
  const gz::sim::Model model(model_entity);
  if (!model.Valid(ecm)) {
    TOBAS_EXIT("Failed to find model.");
  }

  // Initialize rotor models.
  auto rotor_elem = sdf->FindElement(kRotorKey);
  if (!rotor_elem) {
    TOBAS_EXIT("Please specify \"", kRotorKey, "\" elements.");
  }
  while (rotor_elem) {
    IceRotorModel rotor;

    if (!rotor.initialize(rotor_elem, ecm, model)) {
      TOBAS_EXIT("Failed to initialize ICE rotor model.");
    }

    if (rotors_.contains(rotor.getLinkName())) {
      TOBAS_EXIT("Rotor link name \"", rotor.getLinkName(), "\" is duplicated.");
    }

    rotors_[rotor.getLinkName()] = rotor;
    rotor_elem = rotor_elem->GetNextElement(kRotorKey);
  }

  // Initialize engine model.
  const auto engine_elem = sdf->FindElement(kEngineKey);
  if (!engine_elem) {
    TOBAS_EXIT("Please specify \"", kEngineKey, "\" element.");
  }
  if (!engine_.initialize(engine_elem)) {
    TOBAS_EXIT("Failed to initialize engine model.");
  }

  // Register ROS interfaces.
  registerPubSub();
}

void GazeboIcePropulsionSystemPlugin::PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm)
{
  constexpr double kAutoStopTimeout = 0.5;  // [s]

  // Update the previous simulation step time.
  ros2::timeChronoToMsg(info.simTime, prev_sim_time_);

  // Force throttle to zero after a fixed time has elapsed since the last throttle command was issued.
  const auto secs_from_last_cmd = (prev_sim_time_ - last_cmd_time_).seconds();
  if (secs_from_last_cmd > kAutoStopTimeout) {
    engine_.setThrottle(0.0);
  }

  // Update gazebo states.
  for (auto& [_, rotor] : rotors_) {
    rotor.applyWrench(ecm, engine_.getSpeed(), wind_vel_W_);
    rotor.updateJointPosition(ecm, engine_.getPosition());
  }
}

void GazeboIcePropulsionSystemPlugin::PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager&)
{
  // Step simulation
  const auto dt = ch::duration<double>(info.dt).count();
  for (auto& [_, rotor] : rotors_) {
    rotor.step(dt);
  }
  engine_.step(dt);

  // Publish observed states.
  if (publish_state_rate_manager_->update(info.simTime)) {
    for (const auto& [link_name, rotor] : rotors_) {
      auto rotor_state_obs = std::make_unique<tobas_msgs::msg::RotorState>();
      rotor_state_obs->link_name = link_name;
      rotor_state_obs->speed = rotor.getSpeed(engine_.getSpeed());
      rotor_state_obs->thrust = rotor.getThrust(engine_.getSpeed());
      rotor_state_obs->status = tobas_msgs::msg::RotorState::NO_ERROR;
      rotor_state_pubs_.at(link_name)->publish(std::move(rotor_state_obs));
    }

    auto engine_state_obs = std::make_unique<tobas_msgs::msg::EngineState>();
    ros2::timeChronoToMsg(info.simTime, engine_state_obs->header.stamp);
    engine_state_obs->speed = engine_.getSpeed();
    engine_state_obs->fuel_quantity = NAN;    // TODO
    engine_state_obs->oil_temperature = NAN;  // TODO
    engine_state_pub_->publish(std::move(engine_state_obs));
  }

  // Publish ground-truth states.
  for (const auto& [link_name, rotor] : rotors_) {
    auto rotor_state_gt = std::make_unique<tobas_gazebo_msgs::msg::RotorState>();
    ros2::timeChronoToMsg(info.simTime, rotor_state_gt->header.stamp);
    rotor_state_gt->rotation_speed = rotor.getSpeed(engine_.getSpeed());
    rotor_state_gt->current = 0.0;
    rotor_state_gt->vibration_force = 0.0;  // TODO: Vibration model for engine-driven propellers.
    rotor_state_gt_pubs_.at(link_name)->publish(std::move(rotor_state_gt));
  }

  auto engine_state_gt = std::make_unique<tobas_gazebo_msgs::msg::EngineState>();
  ros2::timeChronoToMsg(info.simTime, engine_state_gt->header.stamp);
  engine_state_gt->speed = engine_.getSpeed();
  engine_state_gt->vibration_force = engine_.getVibrationForce();
  engine_state_gt_pub_->publish(std::move(engine_state_gt));
}

void GazeboIcePropulsionSystemPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  constexpr size_t kDefaultPublishStateRate = 100;  // [Hz]

  getSdfParam(sdf, "publishStateRate", publish_state_rate_, kDefaultPublishStateRate, kNonNegative);
}

void GazeboIcePropulsionSystemPlugin::registerPubSub()
{
  latency_pub_ = createPublisher<tobas_msgs::msg::Latency>(topic::kControlLatency);
  engine_state_pub_ = createPublisher<tobas_msgs::msg::EngineState>(topic::kEngineState);
  engine_state_gt_pub_ = createPublisher<tobas_gazebo_msgs::msg::EngineState>(kEngineStateGtTopic);

  for (auto& [link_name, _] : rotors_) {
    rotor_state_pubs_[link_name] =
      createPublisher<tobas_msgs::msg::RotorState>(path::join(kRotorStateTopicNS, link_name));
    rotor_state_gt_pubs_[link_name] =
      createPublisher<tobas_gazebo_msgs::msg::RotorState>(path::join(kRotorStateGtTopicNS, link_name));
  }

  ice_cmd_sub_ = createSubscriber(topic::kIcePropulsionSystemCmd, &self::iceCommandCb, this);
  wind_gt_sub_ = createSubscriber(gazebo::kWindGtTopic, &self::windSpeedGtCb, this);
}

void GazeboIcePropulsionSystemPlugin::iceCommandCb(
  const tobas_msgs::msg::IcePropulsionSystemCommand::ConstSharedPtr& ice_cmd)
{
  constexpr double kThrotLimitMargin = 1e-3;  // [-]

  // Update the time when the last command was received.
  last_cmd_time_ = prev_sim_time_;

  // Update engine throttle.
  const auto& engine_throt = ice_cmd->engine_throttle;
  if (!std::isfinite(engine_throt)) {
    TOBAS_WARN("The commanded engine throttle is not finite: ", engine_throt);
    engine_.setThrottle(engine_throt);
    return;
  }
  if (engine_throt < kMinThrot - kThrotLimitMargin || kMaxThrot + kThrotLimitMargin < engine_throt) {
    TOBAS_WARN("The commanded engine throttle is out of range: ", engine_throt);
  }
  engine_.setThrottle(engine_throt);

  // Update propeller pitch angle.
  for (const auto& elem : ice_cmd->pitch_angles) {
    if (!rotors_.contains(elem.link_name)) {
      TOBAS_WARN("Rotor link \"", elem.link_name, "\" does not exist.");
      continue;
    }
    if (!std::isfinite(elem.angle)) {
      TOBAS_WARN("The commanded pitch angle of propeller \"", elem.link_name, "\" is not finite: ", elem.angle);
      rotors_.at(elem.link_name).setTargetPitchAngle(0.0);
      continue;
    }
    rotors_.at(elem.link_name).setTargetPitchAngle(elem.angle);
  }

  // Publish control latency.
  auto latency = std::make_unique<tobas_msgs::msg::Latency>();
  latency->header.stamp = prev_sim_time_;
  latency->data = prev_sim_time_ - ice_cmd->header.stamp;
  latency_pub_->publish(std::move(latency));
}

void GazeboIcePropulsionSystemPlugin::windSpeedGtCb(const tobas_msgs::Wind::ConstSharedPtr& wind_gt)
{
  vectorKDLToGazebo(wind_gt->vel, wind_vel_W_);
}
}  // namespace gazebo
}  // namespace tobas

GZ_ADD_PLUGIN(
  tobas::gazebo::GazeboIcePropulsionSystemPlugin,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate,
  gz::sim::ISystemPostUpdate)
