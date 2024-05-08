#include "./drone_spider_force_plugin.hpp"
#include "../include/tobas_gazebo_plugins/sdfparam.hpp"

using namespace std;
using namespace ignition::math;

namespace gazebo
{
GazeboDroneSpiderForcePlugin::GazeboDroneSpiderForcePlugin() : super()
{
}

void GazeboDroneSpiderForcePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  gzmsg << "Loading " << kPluginName << "." << endl;

  getSdfParams(sdf);

  link_ = model->GetLink(link_name_);
  if (link_ == nullptr)
    gzthrow(kPluginName << ": Couldn't find specified link \"" << link_name_ << "\".");

  update_connection_ =
    event::Events::ConnectWorldUpdateBegin(boost::bind(&self::onUpdate, this, _1));
}

void GazeboDroneSpiderForcePlugin::getSdfParams(sdf::ElementPtr sdf)
{
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "worldEnd", W_Pos_WP_, Vector3d::Zero);
  getSdfParam(sdf, "droneEnd", B_Pos_BQ_, Vector3d::Zero);
  getSdfParam(sdf, "tension", tension_, kDefaultTension);
}

void GazeboDroneSpiderForcePlugin::onUpdate(const common::UpdateInfo&)
{
  const auto& W_Pos_WB = link_->WorldPose().Pos();
  const auto& W_Rot_B = link_->WorldPose().Rot();
  const auto B_Pos_PQ = W_Rot_B.Inverse() * (W_Pos_WB - W_Pos_WP_) + B_Pos_BQ_;
  const auto axis = -B_Pos_PQ.Normalized();
  const auto force = tension_ * axis;
  link_->AddLinkForce(force, B_Pos_BQ_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboDroneSpiderForcePlugin);
}  // namespace gazebo
