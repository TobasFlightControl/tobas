#include "./tether_station_force_plugin.hpp"
#include "../include/tobas_gazebo_plugins/sdfparam.hpp"

using namespace std;
using namespace ignition::math;

namespace gazebo
{
GazeboTetherStationForcePlugin::GazeboTetherStationForcePlugin() : super()
{
}

void GazeboTetherStationForcePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  gzmsg << "Loading " << kPluginName << "." << endl;

  getSdfParams(sdf);

  link_ = model->GetLink(link_name_);
  if (link_ == nullptr)
    gzthrow(kPluginName << ": Couldn't find specified link \"" << link_name_ << "\".");

  update_connection_ =
    event::Events::ConnectWorldUpdateBegin(boost::bind(&self::onUpdate, this, _1));
}

void GazeboTetherStationForcePlugin::getSdfParams(sdf::ElementPtr sdf)
{
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "worldEnd", W_Pos_WP_, Vector3d::Zero);
  getSdfParam(sdf, "droneEnd", B_Pos_BQ_, Vector3d::Zero);
  getSdfParam(sdf, "tension", tension_, kDefaultTension);
}

void GazeboTetherStationForcePlugin::onUpdate(const common::UpdateInfo&)
{
  const auto& W_Pos_WB = link_->WorldPose().Pos();
  const auto& W_Rot_B = link_->WorldPose().Rot();
  const auto B_Pos_PQ = W_Rot_B.Inverse() * (W_Pos_WB - W_Pos_WP_) + B_Pos_BQ_;
  const auto axis = -B_Pos_PQ.Normalized();
  const auto force = tension_ * axis;  // TODO: 実際張力は一定ではない．より詳細な張力モデルを実装．
  link_->AddLinkForce(force, B_Pos_BQ_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboTetherStationForcePlugin);
}  // namespace gazebo
