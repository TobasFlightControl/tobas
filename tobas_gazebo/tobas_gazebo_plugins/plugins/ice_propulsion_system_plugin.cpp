#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>

#include <tobas_std_tools/range.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/engine_state.hpp>
#include <tobas_msgs/msg/engine_throttle.hpp>
#include <tobas_msgs/msg/propeller_pitch_angle_array.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_tools/utils.hpp>
#include <tobas_gazebo_msgs/msg/throttle.hpp>
#include <tobas_gazebo_msgs/msg/rotor_state.hpp>

#include "../include/tobas_gazebo_plugins/common/common.hpp"
#include "../include/tobas_gazebo_plugins/conversions/conversions.hpp"
#include "../include/tobas_gazebo_plugins/rate_manager.hpp"
#include "../include/tobas_gazebo_plugins/utils.hpp"
#include "../include/tobas_gazebo_plugins/sdf.hpp"

using namespace std;
using namespace chrono;
namespace cmp = gz::sim::components;

namespace gazebo
{
struct Engine
{
  // SDF parameters
  double torque_const;     // [Nm/(rad/s)]
  double friction_torque;  // [Nm]
};

struct Rotor
{
  // SDF parameters
  string link_name;                      // プロペラのリンク名
  int direction;                         // Turning direction: 1(CCW) or -1(CW)
  uint32_t gear_ratio;                   // 減速比 [-]
  tobas_std::Range<double> pitch_range;  // プロペラピッチ角の範囲 [rad]
  std::pair<double, double> motor_const;  // T = (aφ + b) ω^2 (φ: プロペラのピッチ角，ω: プロペラの回転数)
  double moment_const;                    // 反トルク係数 [m]

  // Gazebo objects
  shared_ptr<gz::sim::Joint> joint;
  shared_ptr<gz::sim::Link> link;
  shared_ptr<gz::sim::Link> parent_link;

  // States
  double throttle = 0.;  // [0, 1]
  double velocity = 0.;  // [rad/s]
  double position = 0.;  // [rad]
};

/* Simulates engine and propellers. */
class GazeboICEPropulsionSystemPlugin : public BaseNode,
                                        public gz::sim::System,
                                        public gz::sim::ISystemConfigure,
                                        public gz::sim::ISystemPreUpdate
{
  // Constants
  static constexpr char kRotorKey[] = "rotor";
  static constexpr double kAutoStopTimeout = 0.5;    // [s]
  static constexpr double kThrotLimitMargin = 1e-3;  // [-]

  // Default parameters
  static constexpr size_t kDefaultPublishStateRate = 100;  // [Hz]
  static constexpr double kDefaultMaxModelErrorRate = 0.;

  using self = GazeboICEPropulsionSystemPlugin;

public:
  explicit GazeboICEPropulsionSystemPlugin();

  void Configure(
    const gz::sim::Entity& model_entity,
    const sdf::ElementConstPtr& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager&) override;

  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override;

private:
  Engine engine_;
  map<string, Rotor> rotors_;
  size_t publish_state_rate_;  // [Hz]

  gz::math::Vector3d wind_vel_W_ = gz::math::Vector3d::Zero;  // [m/s]
  steady_clock::duration prev_sim_time_;
  steady_clock::duration last_cmd_time_;  // 最後にスロットルコマンドが指令された時刻
  bool wind_received_ = false;
  RateManager::SharedPtr publish_state_rate_manager_;

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::EngineState> engine_state_pub_;
  ros2::PublisherPtr<tobas_msgs::msg::RotorStateArray> rotor_states_pub_;

  // Subscribers
  ros2::SubscriberPtr<tobas_msgs::msg::EngineThrottle> engine_throttle_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::PropellerPitchAngleArray> propeller_pitches_sub_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
  void registerPubSub();
  void addModelError();

  void engineThrottleCb(const tobas_msgs::msg::EngineThrottle::ConstSharedPtr& engine_throttle);
  void propellerPitchesCb(const tobas_msgs::msg::PropellerPitchAngleArray::ConstSharedPtr& propeller_pitches);
};

GazeboICEPropulsionSystemPlugin::GazeboICEPropulsionSystemPlugin()
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
  addModelError();

  // Get robot model
  gz::sim::Model model(model_entity);
  if (!model.Valid(ecm))
    TOBAS_EXIT("Failed to find model.");

  // Get gazebo objects
  for (auto& [link_name, rotor] : rotors_)
  {
    // Get joint
    const auto joint_entity = findJointWithChildLink(ecm, link_name);
    if (!joint_entity.has_value())
      TOBAS_EXIT("Failed to find the parent joint of rotor link \"", link_name, "\".");
    rotor.joint = make_shared<gz::sim::Joint>(joint_entity.value());
    if (!rotor.joint->Valid(ecm))
      TOBAS_EXIT("Failed to find rotor link \"", link_name, "\".");

    // Get joint name
    const auto joint_name = rotor.joint->Name(ecm).value();

    // Check joint type
    const auto joint_type = rotor.joint->Type(ecm).value();
    if (joint_type != sdf::JointType::CONTINUOUS && joint_type != sdf::JointType::REVOLUTE)
      TOBAS_EXIT("Joint \"", joint_name, "\" is not a rotating joint.");

    // Get child link
    const auto link_entity = model.LinkByName(ecm, link_name);
    rotor.link = make_shared<gz::sim::Link>(link_entity);
    if (!rotor.link->Valid(ecm))
      TOBAS_EXIT("Failed to find the child link \"", link_name, "\".");

    // Get parent link
    const auto parent_link_name = rotor.joint->ChildLinkName(ecm).value();
    const auto parent_link_entity = model.LinkByName(ecm, parent_link_name);
    rotor.parent_link = make_shared<gz::sim::Link>(parent_link_entity);
    if (!rotor.parent_link->Valid(ecm))
      TOBAS_EXIT("Failed to find the parent link \"", parent_link_name, "\".");

    // Create necessary components
    if (!getComponent<cmp::JointAxis>(joint_entity.value(), ecm))
      TOBAS_EXIT("Failed to get component JointAxis of joint \"", joint_name, "\".");
    if (!getComponent<cmp::JointVelocity>(joint_entity.value(), ecm))
      TOBAS_EXIT("Failed to get component JointVelocity of joint \"", joint_name, "\".");
    if (!getComponent<cmp::WorldPose>(link_entity, ecm))
      TOBAS_EXIT("Failed to get component WorldPose of link \"", link_name, "\".");
    if (!getComponent<cmp::WorldLinearVelocity>(link_entity, ecm))
      TOBAS_EXIT("Failed to get component WorldLinearVelocity of link \"", link_name, "\".");
  }

  // Register ROS interfaces
  registerPubSub();
}

void GazeboICEPropulsionSystemPlugin::PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm)
{
  (void)info;
  (void)ecm;
  // TODO
}

void GazeboICEPropulsionSystemPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  // Engine
  getSdfParam(sdf, "torqueConstant", engine_.torque_const, POSITIVE);
  getSdfParam(sdf, "dynamicFrictionTorque", engine_.friction_torque, POSITIVE);

  // Rotors
  auto rotor_elem = sdf->FindElement(kRotorKey);
  while (rotor_elem)
  {
    Rotor rotor;

    getSdfParam(rotor_elem, "linkName", rotor.link_name);
    if (rotors_.contains(rotor.link_name))
      TOBAS_EXIT("Rotor link name \"", rotor.link_name, "\" is duplicated.");

    if (!getTurningDirection(rotor_elem, rotor.direction))
      TOBAS_EXIT("Failed to get turning direction of rotor \"", rotor.link_name, "\".");

    getSdfParam(rotor_elem, "gearRatio", rotor.gear_ratio, POSITIVE);

    getSdfParam(rotor_elem, "minPitchAngle", rotor.pitch_range.lower);
    getSdfParam(rotor_elem, "maxPitchAngle", rotor.pitch_range.upper);
    if (!rotor.pitch_range.isValid())
      TOBAS_EXIT("Pitch range of rotor \"", rotor.link_name, "\" is invalid.");

    getSdfParam(rotor_elem, "motorConstant", rotor.motor_const);
    if (rotor.motor_const.first <= 0.)
      TOBAS_EXIT("The first term of motor constant must be positive.");

    getSdfParam(rotor_elem, "momentConstant", rotor.moment_const, POSITIVE);

    rotors_[rotor.link_name] = rotor;
    rotor_elem = sdf->GetNextElement(kRotorKey);
  }

  // Other
  getSdfParam(sdf, "publishStateRate", publish_state_rate_, kDefaultPublishStateRate, NON_NEGATIVE);
}

void GazeboICEPropulsionSystemPlugin::registerPubSub()
{
  engine_state_pub_ = createPublisher<tobas_msgs::msg::EngineState>(tobas::kEngineStateTopic);
  rotor_states_pub_ = createPublisher<tobas_msgs::msg::RotorStateArray>(tobas::kRotorStatesTopic);

  engine_throttle_sub_ = createSubscriber(tobas::kEngineThrottleCmdTopic, &self::engineThrottleCb, this);
  propeller_pitches_sub_ = createSubscriber(tobas::kPropellerPitchesCmdTopic, &self::propellerPitchesCb, this);
}

void GazeboICEPropulsionSystemPlugin::addModelError()
{
  // TODO
}

void GazeboICEPropulsionSystemPlugin::engineThrottleCb(
  const tobas_msgs::msg::EngineThrottle::ConstSharedPtr& engine_throttle)
{
  // TODO
}

void GazeboICEPropulsionSystemPlugin::propellerPitchesCb(
  const tobas_msgs::msg::PropellerPitchAngleArray::ConstSharedPtr& propeller_pitches)
{
  // TODO
}
}  // namespace gazebo

GZ_ADD_PLUGIN(
  gazebo::GazeboICEPropulsionSystemPlugin,
  gz::sim::System,
  gazebo::GazeboICEPropulsionSystemPlugin::ISystemConfigure,
  gazebo::GazeboICEPropulsionSystemPlugin::ISystemPreUpdate)
