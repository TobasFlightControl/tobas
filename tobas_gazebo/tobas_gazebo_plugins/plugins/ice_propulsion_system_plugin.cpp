#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/engine_state.hpp>
#include <tobas_msgs/msg/ice_propulsion_system_command.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_msgs_adapter/wind.hpp>

#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_msgs/msg/throttle.hpp>
#include <tobas_gazebo_msgs/msg/rotor_state.hpp>

#include "../include/tobas_gazebo_plugins/common/common.hpp"
#include "../include/tobas_gazebo_plugins/conversions/conversions.hpp"
#include "../include/tobas_gazebo_plugins/rate_manager.hpp"
#include "../include/tobas_gazebo_plugins/sdf.hpp"
#include "../include/tobas_gazebo_plugins/ice_rotor_model.hpp"
#include "../include/tobas_gazebo_plugins/engine_model.hpp"

using namespace std;
using namespace chrono;

namespace gazebo
{
/* Simulates engine and propellers. */
class GazeboICEPropulsionSystemPlugin : public BaseNode,
                                        public gz::sim::System,
                                        public gz::sim::ISystemConfigure,
                                        public gz::sim::ISystemPreUpdate,
                                        public gz::sim::ISystemPostUpdate
{
  // Constants
  static constexpr char kRotorKey[] = "rotor";
  static constexpr char kEngineKey[] = "engine";
  static constexpr double kAutoStopTimeout = 0.5;    // [s]
  static constexpr double kThrotLimitMargin = 1e-3;  // [-]

  // Default parameters
  static constexpr size_t kDefaultPublishStateRate = 100;  // [Hz]

  using self = GazeboICEPropulsionSystemPlugin;

public:
  explicit GazeboICEPropulsionSystemPlugin();

  void Configure(
    const gz::sim::Entity& model_entity,
    const sdf::ElementConstPtr& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager&) override;

  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override;
  void PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) override;

private:
  ICERotorModelMap rotors_;
  EngineModel engine_;

  // SDF parameters
  size_t publish_state_rate_;  // [Hz]

  gz::math::Vector3d wind_vel_W_ = gz::math::Vector3d::Zero;  // [m/s]
  steady_clock::duration prev_sim_time_;
  steady_clock::duration last_cmd_time_;  // 最後にスロットルコマンドが指令された時刻
  RateManager::SharedPtr publish_state_rate_manager_;

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::EngineState> engine_state_pub_;
  map<string, ros2::PublisherPtr<tobas_msgs::msg::RotorState>> rotor_state_pubs_;
  map<string, ros2::PublisherPtr<tobas_gazebo_msgs::msg::RotorState>> rotor_state_gt_pubs_;

  // Subscribers
  ros2::SubscriberPtr<tobas_msgs::msg::IcePropulsionSystemCommand> ice_cmd_sub_;
  ros2::SubscriberPtr<tobas_msgs::Wind> wind_gt_sub_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
  void registerPubSub();

  void iceCommandCb(const tobas_msgs::msg::IcePropulsionSystemCommand::ConstSharedPtr& ice_cmd);
  void windSpeedGtCb(const tobas_msgs::Wind::ConstSharedPtr& wind_gt);
};

GazeboICEPropulsionSystemPlugin::GazeboICEPropulsionSystemPlugin() : engine_(rotors_)
{
}

void GazeboICEPropulsionSystemPlugin::Configure(
  const gz::sim::Entity& model_entity,
  const sdf::ElementConstPtr& sdf,
  gz::sim::EntityComponentManager& ecm,
  gz::sim::EventManager&)
{
  initialize("gazebo_ice_propulsion_system_plugin", sdf);
  getSdfParams(sdf);

  publish_state_rate_manager_ = make_shared<RateManager>(publish_state_rate_);

  // Get robot model
  gz::sim::Model model(model_entity);
  if (!model.Valid(ecm))
    TOBAS_EXIT("Failed to find model.");

  // Initialize rotor models
  auto rotor_elem = sdf->FindElement(kRotorKey);
  if (!rotor_elem)
    TOBAS_EXIT("Please specify \"", kRotorKey, "\" elements.");
  while (rotor_elem)
  {
    ICERotorModel rotor;

    if (!rotor.initialize(rotor_elem, ecm, model))
      TOBAS_EXIT("Failed to initialize ICE rotor model.");

    if (rotors_.contains(rotor.getLinkName()))
      TOBAS_EXIT("Rotor link name \"", rotor.getLinkName(), "\" is duplicated.");

    rotors_[rotor.getLinkName()] = rotor;
    rotor_elem = rotor_elem->GetNextElement(kRotorKey);
  }

  // Initialize engine model
  const auto engine_elem = sdf->FindElement(kEngineKey);
  if (!engine_elem)
    TOBAS_EXIT("Please specify \"", kEngineKey, "\" element.");
  if (!engine_.initialize(engine_elem))
    TOBAS_EXIT("Failed to initialize engine model.");

  // Register ROS interfaces
  registerPubSub();
}

void GazeboICEPropulsionSystemPlugin::PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm)
{
  // Update the previous simulation step time
  prev_sim_time_ = info.simTime;

  // 最後にスロットルコマンドが指令された時刻から一定時間経過したら強制的にスロットルをゼロにする
  const auto secs_from_last_cmd = duration<double>(info.simTime - last_cmd_time_).count();
  if (secs_from_last_cmd > kAutoStopTimeout)
    engine_.setThrottle(0.);

  // Update gazebo states
  for (auto& [_, rotor] : rotors_)
  {
    rotor.applyWrench(ecm, engine_.getSpeed(), wind_vel_W_);
    rotor.updateJointPosition(ecm, engine_.getPosition());
  }
}

void GazeboICEPropulsionSystemPlugin::PostUpdate(
  const gz::sim::UpdateInfo& info,
  const gz::sim::EntityComponentManager&)
{
  // Step simulation
  const auto dt = duration<double>(info.dt).count();
  for (auto& [_, rotor] : rotors_)
    rotor.step(dt);
  engine_.step(dt);

  // Publish observed states
  if (publish_state_rate_manager_->update(info.simTime))
  {
    for (const auto& [link_name, rotor] : rotors_)
    {
      auto state_msg_obs = make_unique<tobas_msgs::msg::RotorState>();
      state_msg_obs->link_name = link_name;
      state_msg_obs->speed = rotor.getSpeed(engine_.getSpeed());
      state_msg_obs->thrust = rotor.getThrust(engine_.getSpeed());
      state_msg_obs->status = tobas_msgs::msg::RotorState::NO_ERROR;
      rotor_state_pubs_.at(link_name)->publish(move(state_msg_obs));
    }

    auto engine_state = make_unique<tobas_msgs::msg::EngineState>();
    ros2::timeChronoToMsg(info.simTime, engine_state->header.stamp);
    engine_state->fuel_quantity = NAN;    // TODO
    engine_state->oil_temperature = NAN;  // TODO
    engine_state_pub_->publish(move(engine_state));
  }

  // Publish ground-truth states
  for (const auto& [link_name, rotor] : rotors_)
  {
    auto state_msg_gt = make_unique<tobas_gazebo_msgs::msg::RotorState>();
    ros2::timeChronoToMsg(info.simTime, state_msg_gt->header.stamp);
    state_msg_gt->rotation_speed = rotor.getSpeed(engine_.getSpeed());
    state_msg_gt->current = 0.;
    state_msg_gt->rotor_noise = 0.;  // TODO: エンジン駆動プロペラの振動モデル
    rotor_state_gt_pubs_.at(link_name)->publish(move(state_msg_gt));
  }
}

void GazeboICEPropulsionSystemPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "publishStateRate", publish_state_rate_, kDefaultPublishStateRate, NON_NEGATIVE);
}

void GazeboICEPropulsionSystemPlugin::registerPubSub()
{
  engine_state_pub_ = createPublisher<tobas_msgs::msg::EngineState>(tobas::kEngineStateTopic);

  for (auto& [link_name, _] : rotors_)
  {
    rotor_state_pubs_[link_name] =
      createPublisher<tobas_msgs::msg::RotorState>(path::join(kRotorStateTopicNS, link_name));
    rotor_state_gt_pubs_[link_name] =
      createPublisher<tobas_gazebo_msgs::msg::RotorState>(path::join(kRotorStateGtTopicNS, link_name));
  }

  ice_cmd_sub_ = createSubscriber(tobas::kIcePropulsionSystemCmdTopic, &self::iceCommandCb, this);
  wind_gt_sub_ = createSubscriber(gazebo::kWindGtTopic, &self::windSpeedGtCb, this);
}

void GazeboICEPropulsionSystemPlugin::iceCommandCb(
  const tobas_msgs::msg::IcePropulsionSystemCommand::ConstSharedPtr& ice_cmd)
{
  // 最後にコマンドを受け取った時刻を更新
  last_cmd_time_ = prev_sim_time_;

  // エンジンスロットルを更新
  const auto& engine_throt = ice_cmd->engine_throttle;
  if (engine_throt < tobas::kMinThrot - kThrotLimitMargin || tobas::kMaxThrot + kThrotLimitMargin < engine_throt)
    TOBAS_ERROR("The commanded throttle ", engine_throt, " is out of range.");
  engine_.setThrottle(engine_throt);

  // プロペラピッチ角を更新
  for (const auto& elem : ice_cmd->pitch_angles)
  {
    if (!rotors_.contains(elem.link_name))
    {
      TOBAS_WARN("Rotor link \"", elem.link_name, "\" does not exist.");
      continue;
    }
    rotors_.at(elem.link_name).setTargetPitchAngle(elem.angle);
  }
}

void GazeboICEPropulsionSystemPlugin::windSpeedGtCb(const tobas_msgs::Wind::ConstSharedPtr& wind_gt)
{
  vectorKDLToGazebo(wind_gt->vel, wind_vel_W_);
}
}  // namespace gazebo

GZ_ADD_PLUGIN(
  gazebo::GazeboICEPropulsionSystemPlugin,
  gz::sim::System,
  gazebo::GazeboICEPropulsionSystemPlugin::ISystemConfigure,
  gazebo::GazeboICEPropulsionSystemPlugin::ISystemPreUpdate,
  gazebo::GazeboICEPropulsionSystemPlugin::ISystemPostUpdate)
