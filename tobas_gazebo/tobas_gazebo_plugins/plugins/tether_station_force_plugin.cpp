#include "./tether_station_force_plugin.hpp"
#include "../include/tobas_gazebo_plugins/common.hpp"
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

  model_ = model;

  link_ = model->GetLink(link_name_);
  if (link_ == nullptr)
    gzthrow(kPluginName << ": Couldn't find specified link \"" << link_name_ << "\".");

  contacts_sub_ = nh_.subscribe("/" + ns_ + "/" + kContactStatesTopic, 1, &self::contactStatesCb, this);

  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&self::onUpdate, this, _1));
}

void GazeboTetherStationForcePlugin::getSdfParams(sdf::ElementPtr sdf)
{
  getSdfParam(sdf, "robotNamespace", ns_);
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "worldEnd", W_Pos_WP_, Vector3d::Zero);
  getSdfParam(sdf, "droneEnd", B_Pos_BQ_, Vector3d::Zero);
  getSdfParam(sdf, "tension", tension_, kDefaultTension);
}

void GazeboTetherStationForcePlugin::onUpdate(const common::UpdateInfo& info)
{
  if (contacts_ == nullptr)
  {
    GZ_WARN_THROTTLE(kWarnPeriod, kPluginName << ": /" << ns_ << "/" << kContactStatesTopic << " is not received yet.");
    return;
  }

  // 最初に接地するまでは張力を加えない
  // 初期位置がずれると，接触が不安定になり加速度に異常が生じる恐れがある．
  if (!first_contact_detected_)
  {
    if (isContactWithPlane())
    {
      gzmsg << kPluginName << ": First contact with plane is detected." << endl;
      first_contact_detected_ = true;
    }
    return;
  }

  // 接地時は張力を加えない
  // 接地時に張力を加えると，接触が不安定になり加速度に異常が生じる恐れがある．
  if (isContactWithPlane())
    return;

  // ケーブルの端点間ベクトルを計算
  const auto& W_Pos_WB = link_->WorldPose().Pos();
  const auto& W_Rot_B = link_->WorldPose().Rot();
  const auto B_Pos_PQ = W_Rot_B.Inverse() * (W_Pos_WB - W_Pos_WP_) + B_Pos_BQ_;

  // ケーブルの方向に張力を加える
  const auto axis = -B_Pos_PQ.Normalized();
  const auto force = tension_ * axis;  // TODO: 実際張力は一定ではない．より詳細な張力モデルを実装．
  link_->AddLinkForce(force, B_Pos_BQ_);
}

bool GazeboTetherStationForcePlugin::isThis(const std::string& name)
{
  return name == model_->GetName();
}

bool GazeboTetherStationForcePlugin::isPlane(const uint32_t& shape)
{
  return ((shape & physics::Base::SHAPE) > 0 && (shape & physics::Base::PLANE_SHAPE) > 0);
}

bool GazeboTetherStationForcePlugin::isContactWithPlane()
{
  for (const auto state : contacts_->states)
  {
    const auto& col1 = state.collision1;
    const auto& col2 = state.collision2;
    if ((isThis(col1.name) && isPlane(col2.shape_type)) || (isThis(col2.name) && isPlane(col1.shape_type)))
      return true;
  }

  return false;
}

void GazeboTetherStationForcePlugin::contactStatesCb(const tobas_gazebo_msgs::ContactStatesConstPtr& contacts)
{
  contacts_ = contacts;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboTetherStationForcePlugin);
}  // namespace gazebo
