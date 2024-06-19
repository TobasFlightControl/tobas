#pragma once

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
class GazeboTetherStationForcePlugin : public ModelPlugin
{
  // Constants
  static constexpr char kPluginName[] = "tether_station_force_plugin";

  // Default parameters
  static constexpr double kDefaultTension = 1.;  // [N]

  using self = GazeboTetherStationForcePlugin;
  using super = ModelPlugin;

public:
  explicit GazeboTetherStationForcePlugin();

protected:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  // SDF parameters
  std::string link_name_;
  ignition::math::Vector3d W_Pos_WP_;
  ignition::math::Vector3d B_Pos_BQ_;
  double tension_;  // [N]

  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;

  void getSdfParams(sdf::ElementPtr sdf);
  void onUpdate(const common::UpdateInfo& info);
};
}  // namespace gazebo
