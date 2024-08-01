#include "./tether_station_force_plugin.hpp"
#include "../include/tobas_gazebo_plugins/common.hpp"
#include "../include/tobas_gazebo_plugins/sdfparam.hpp"
#include "../include/tobas_gazebo_plugins/utils.hpp"

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

  params_.tension = init_tension_;
  params_.maximum_length = init_max_length_;

  model_ = model;

  link_ = model->GetLink(link_name_);
  if (link_ == nullptr)
    gzthrow(kPluginName << ": Couldn't find specified link \"" << link_name_ << "\".");

  contacts_sub_ = node_.subscribe("/" + ns_ + "/" + kContactStatesTopic, 1, &self::contactStatesCb, this);
  get_params_ss_ = node_.advertiseService("/" + ns_ + "/" + kGetTetherParamsSrv, &self::getParamsCb, this);
  set_params_ss_ = node_.advertiseService("/" + ns_ + "/" + kSetTetherParamsSrv, &self::setParamsCb, this);

  update_connection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&self::onUpdate, this, _1));
}

void GazeboTetherStationForcePlugin::getSdfParams(sdf::ElementPtr sdf)
{
  getSdfParam(sdf, "robotNamespace", ns_);
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "worldEnd", W_Pos_WP_, zero3);
  getSdfParam(sdf, "droneEnd", B_Pos_BQ_, zero3);
  getSdfParam(sdf, "initialTension", init_tension_, kDefaultInitTension, NON_NEGATIVE);
  getSdfParam(sdf, "initialMaximumLength", init_max_length_, kDefaultInitMaxLength, POSITIVE);
  getSdfParam(sdf, "youngModulus", young_, kDefaultYoungModulus, POSITIVE);
  getSdfParam(sdf, "crossSectionArea", csa_, kDefaultCrossSectionArea, POSITIVE);
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

  // 現在の状態を取得
  const auto& W_Pos_WB = link_->WorldPose().Pos();
  const auto& W_Rot_B = link_->WorldPose().Rot();
  const auto W_Vel_WB = link_->WorldLinearVel();
  const auto W_Gyro_WB = link_->WorldAngularVel();

  // ケーブルの端点間ベクトルを計算
  const auto W_Pos_BQ = W_Rot_B * B_Pos_BQ_;
  const auto W_Pos_PQ = W_Pos_WB - W_Pos_WP_ + W_Pos_BQ;
  const auto B_Pos_PQ = W_Rot_B.Inverse() * (W_Pos_WB - W_Pos_WP_) + B_Pos_BQ_;

  // ケーブルが伸び切っていない場合は一定張力
  auto T = params_.tension;  // [N]

  // ケーブル長が限界以上ならばワイヤロープの弾粘性モデル
  const auto length = B_Pos_PQ.Length();  // [m]
  if (length > params_.maximum_length)
  {
    // ケーブル長の変位を計算
    const auto x = length - params_.maximum_length;  // [m]

    // ケーブル長の変化率を計算
    const auto W_Vel_PQ = W_Vel_WB + W_Gyro_WB.Cross(W_Pos_BQ);  // [m]
    const auto xd = W_Vel_PQ.Dot(W_Pos_PQ) / length;             // [m/s]

    // マスバネダンパ系の係数
    const auto m = computeTotalMass(model_);                // [kg]
    const auto k = young_ * csa_ * params_.maximum_length;  // [N/m]
    const auto d = 2 * sqrt(m * k);                         // [Ns/m] 臨海減衰する粘性係数

    // ケーブルにかかる力を計算
    T = k * x + d * xd;  // [N]
  }

  // ケーブルの方向に張力を加える
  const auto axis = -B_Pos_PQ.Normalized();
  const auto force = T * axis;
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
  for (const auto& state : contacts_->states)
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

bool GazeboTetherStationForcePlugin::getParamsCb(
  tobas_gazebo_msgs::GetTetherParamsRequest& req,
  tobas_gazebo_msgs::GetTetherParamsResponse& res)
{
  res.params = params_;
  return true;
}

bool GazeboTetherStationForcePlugin::setParamsCb(
  tobas_gazebo_msgs::SetTetherParamsRequest& req,
  tobas_gazebo_msgs::SetTetherParamsResponse& res)
{
  res.params = params_;

  // Tension
  if (req.params.tension < 0)
  {
    gzerr << kPluginName << ": Tension must be non-negative." << endl;
    res.success = false;
    return true;
  }
  params_.tension = res.params.tension = req.params.tension;

  // Maximum length
  if (req.params.maximum_length <= 0)
  {
    gzerr << kPluginName << ": Maximum length must be positive." << endl;
    res.success = false;
    return true;
  }
  params_.maximum_length = res.params.maximum_length = req.params.maximum_length;

  res.success = true;
  return true;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboTetherStationForcePlugin);
}  // namespace gazebo
