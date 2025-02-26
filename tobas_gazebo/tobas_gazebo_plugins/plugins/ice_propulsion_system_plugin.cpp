#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>

#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_tools/utils.hpp>
#include <tobas_gazebo_msgs/msg/throttle.hpp>
#include <tobas_gazebo_msgs/msg/rotor_state.hpp>

#include "../include/tobas_gazebo_plugins/common/common.hpp"
#include "../include/tobas_gazebo_plugins/conversions/conversions.hpp"
#include "../include/tobas_gazebo_plugins/rate_manager.hpp"
#include "../include/tobas_gazebo_plugins/utils.hpp"

using namespace std;
namespace cmp = gz::sim::components;

namespace gazebo
{
/* Simulates engine and propellers. */
class GazeboICEPropulsionSystemPlugin : public BaseNode,
                                        public gz::sim::System,
                                        public gz::sim::ISystemConfigure,
                                        public gz::sim::ISystemPreUpdate
{
  // Constants
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
  // SDF parameters
  double torque_const_;     // [Nm/(rad/s)]
  double friction_torque_;  // [Nm]

  void getSdfParams(const sdf::ElementConstPtr& sdf);
  void registerPubSub();
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

  // Get robot model
  gz::sim::Model model(model_entity);
  if (!model.Valid(ecm))
    TOBAS_EXIT("Failed to find model.");

  // Get propeller joint models
  // TODO
}

void GazeboICEPropulsionSystemPlugin::PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm)
{
  (void)info;
  (void)ecm;
  // TODO
}

void GazeboICEPropulsionSystemPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "torqueConstant", torque_const_, POSITIVE);
  getSdfParam(sdf, "dynamicFrictionTorque", friction_torque_, POSITIVE);

  // Propellers
  // TODO
}

void GazeboICEPropulsionSystemPlugin::registerPubSub()
{
  // TODO
}
}  // namespace gazebo

GZ_ADD_PLUGIN(
  gazebo::GazeboICEPropulsionSystemPlugin,
  gz::sim::System,
  gazebo::GazeboICEPropulsionSystemPlugin::ISystemConfigure,
  gazebo::GazeboICEPropulsionSystemPlugin::ISystemPreUpdate)
