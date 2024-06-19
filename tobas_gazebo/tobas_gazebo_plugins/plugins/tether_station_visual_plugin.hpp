#pragma once

#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/DynamicLines.hh>

namespace gazebo
{
class GazeboTetherStationVisualPlugin : public VisualPlugin
{
  // Constants
  static constexpr char kPluginName[] = "tether_station_visual_plugin";

  using self = GazeboTetherStationVisualPlugin;
  using super = VisualPlugin;

public:
  explicit GazeboTetherStationVisualPlugin();
  ~GazeboTetherStationVisualPlugin();

protected:
  void Load(rendering::VisualPtr visual, sdf::ElementPtr sdf) override;

private:
  // SDF parameters
  ignition::math::Vector3d W_Pos_WP_;
  ignition::math::Vector3d B_Pos_BQ_;

  event::ConnectionPtr update_connection_;
  rendering::VisualPtr visual_;
  rendering::DynamicLines* line_;

  void getSdfParams(sdf::ElementPtr sdf);
  void onUpdate();
};
}  // namespace gazebo
