#include "./tether_station_visual_plugin.hpp"


using namespace std;
using namespace gz::math;

namespace gazebo
{
GazeboTetherStationVisualPlugin::GazeboTetherStationVisualPlugin() : super()
{
}

GazeboTetherStationVisualPlugin::~GazeboTetherStationVisualPlugin()
{
  visual_->DeleteDynamicLine(line_);
}

void GazeboTetherStationVisualPlugin::Load(rendering::VisualPtr visual, sdf::ElementPtr sdf)
{


  getSdfParams(sdf);

  visual_ = visual;
  visual_->SetVisible(true);

  line_ = visual_->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
  Color color(1, 1, 1, 1);
  line_->AddPoint(Vector3d::Zero, color);
  line_->AddPoint(B_Pos_BQ_, color);

  update_connection_ = event::Events::ConnectPreRender(std::bind(&self::onUpdate, this));
}

void GazeboTetherStationVisualPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "worldEnd", W_Pos_WP_, Vector3d::Zero);
  getSdfParam(sdf, "droneEnd", B_Pos_BQ_, Vector3d::Zero);
}

void GazeboTetherStationVisualPlugin::onUpdate()
{
  const auto& W_Pos_WB = visual_->WorldPose().Pos();
  const auto& W_Rot_B = visual_->WorldPose().Rot();
  const auto B_Pos_BP = W_Rot_B.Inverse() * (W_Pos_WP_ - W_Pos_WB);
  line_->SetPoint(0, B_Pos_BP);
}

GZ_REGISTER_VISUAL_PLUGIN(GazeboTetherStationVisualPlugin);
}  // namespace gazebo
