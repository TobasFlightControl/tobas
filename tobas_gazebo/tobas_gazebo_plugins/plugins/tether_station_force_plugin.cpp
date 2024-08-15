#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>

#include <tobas_path_tools/join.hpp>

#include <tobas_gazebo_tools/model_mass_holder.hpp>
#include <tobas_gazebo_msgs/srv/get_tether_params.hpp>
#include <tobas_gazebo_msgs/srv/set_tether_params.hpp>

#include "../include/tobas_gazebo_plugins/common/common.hpp"
#include "../include/tobas_gazebo_plugins/utils.hpp"

using namespace std;
using namespace gz;
namespace cmp = sim::components;

namespace gazebo
{
class GazeboTetherStationForcePlugin : public BaseNode,
                                       public sim::System,
                                       public sim::ISystemConfigure,
                                       public sim::ISystemPreUpdate
{
  // Default parameters
  static constexpr double kDefaultInitTension = 1.;       // [N]
  static constexpr double kDefaultInitMaxLength = 5.;     // [N]
  static constexpr double kDefaultYoungModulus = 200.;    // [MPa] 低密度ポリエチレン
  static constexpr double kDefaultCrossSectionArea = 1.;  // [mm^2]

  using self = GazeboTetherStationForcePlugin;
  using GetSrv = tobas_gazebo_msgs::srv::GetTetherParams;
  using SetSrv = tobas_gazebo_msgs::srv::SetTetherParams;

public:
  explicit GazeboTetherStationForcePlugin();

  void Configure(
    const sim::Entity& model_entity,
    const sdf::ElementConstPtr& sdf,
    sim::EntityComponentManager& ecm,
    sim::EventManager&) override;

  void PreUpdate(const sim::UpdateInfo& info, sim::EntityComponentManager& ecm) override;

private:
  // SDF parameters
  std::string link_name_;
  gz::math::Vector3d W_Pos_WP_;
  gz::math::Vector3d B_Pos_BQ_;
  double init_tension_;     // [N]
  double init_max_length_;  // [m]
  double young_;            // [MPa] ヤング率 (Young Modulus)
  double csa_;              // [mm^2] 断面積 (Cross-Sectional Area)

  shared_ptr<sim::Model> model_;
  shared_ptr<sim::Link> link_;

  const cmp::WorldPose* pose_W_;
  const cmp::WorldLinearVelocity* vel_W_;
  const cmp::WorldAngularVelocity* gyro_W_;

  string model_name_;
  ModelMassHolder mass_holder_;

  tobas_gazebo_msgs::msg::TetherParams params_;

  ServicePtr<GetSrv> get_params_ss_;
  ServicePtr<SetSrv> set_params_ss_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);

  void getParamsCb(const GetSrv::Request::ConstSharedPtr& req, const GetSrv::Response::SharedPtr& res);
  void setParamsCb(const SetSrv::Request::ConstSharedPtr& req, const SetSrv::Response::SharedPtr& res);
};

GazeboTetherStationForcePlugin::GazeboTetherStationForcePlugin() : BaseNode("tether_station_force_plugin")
{
}

void GazeboTetherStationForcePlugin::Configure(
  const sim::Entity& model_entity,
  const sdf::ElementConstPtr& sdf,
  sim::EntityComponentManager& ecm,
  sim::EventManager&)
{
  initialize(sdf);
  getSdfParams(sdf);

  params_.tension = init_tension_;
  params_.maximum_length = init_max_length_;

  model_ = make_shared<sim::Model>(model_entity);
  if (!model_->Valid(ecm))
    TOBAS_EXIT("Failed to find model.");

  const auto link_entity = model_->LinkByName(ecm, link_name_);
  link_ = make_shared<sim::Link>(link_entity);
  if (!link_->Valid(ecm))
    TOBAS_EXIT("Failed to find specified link \"", link_name_, "\".");

  pose_W_ = getComponent<cmp::WorldPose>(link_entity, ecm);
  vel_W_ = getComponent<cmp::WorldLinearVelocity>(link_entity, ecm);
  gyro_W_ = getComponent<cmp::WorldAngularVelocity>(link_entity, ecm);

  model_name_ = model_->Name(ecm);

  if (!mass_holder_.initialize(model_entity, ecm))
    TOBAS_EXIT("Failed to initialize model mass holder.");

  get_params_ss_ = createService<GetSrv>(path::join(ns(), kGetTetherParamsSrv), &self::getParamsCb, this);
  set_params_ss_ = createService<SetSrv>(path::join(ns(), kSetTetherParamsSrv), &self::setParamsCb, this);
}

void GazeboTetherStationForcePlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "worldEnd", W_Pos_WP_, math::Vector3d::Zero);
  getSdfParam(sdf, "droneEnd", B_Pos_BQ_, math::Vector3d::Zero);
  getSdfParam(sdf, "initialTension", init_tension_, kDefaultInitTension, NON_NEGATIVE);
  getSdfParam(sdf, "initialMaximumLength", init_max_length_, kDefaultInitMaxLength, POSITIVE);
  getSdfParam(sdf, "youngModulus", young_, kDefaultYoungModulus, POSITIVE);
  getSdfParam(sdf, "crossSectionArea", csa_, kDefaultCrossSectionArea, POSITIVE);
}

void GazeboTetherStationForcePlugin::PreUpdate(const sim::UpdateInfo&, sim::EntityComponentManager& ecm)
{
  // 現在の状態を取得
  const auto& W_Pos_WB = pose_W_->Data().Pos();
  const auto& W_Rot_B = pose_W_->Data().Rot();
  const auto& W_Vel_WB = vel_W_->Data();
  const auto& W_Gyro_WB = gyro_W_->Data();

  // ケーブルの端点間ベクトルを計算
  const auto W_Pos_BQ = W_Rot_B.RotateVector(B_Pos_BQ_);
  const auto W_Pos_PQ = W_Pos_WB - W_Pos_WP_ + W_Pos_BQ;

  // ケーブルが伸び切っていない場合は一定張力
  auto T = params_.tension;  // [N]

  // ケーブル長が限界以上ならばワイヤロープの弾粘性モデル
  const auto length = W_Pos_PQ.Length();  // [m]
  if (length > params_.maximum_length)
  {
    // ケーブル長の変位を計算
    const auto x = length - params_.maximum_length;  // [m]

    // ケーブル長の変化率を計算
    const auto W_Vel_PQ = W_Vel_WB + W_Gyro_WB.Cross(W_Pos_BQ);  // [m]
    const auto xd = W_Vel_PQ.Dot(W_Pos_PQ) / length;             // [m/s]

    // マスバネダンパ系の係数
    const auto m = mass_holder_.getMass();                  // [kg]
    const auto k = young_ * csa_ * params_.maximum_length;  // [N/m]
    const auto d = 2 * sqrt(m * k);                         // [Ns/m] 臨海減衰する粘性係数

    // ケーブルにかかる力を計算
    T = k * x + d * xd;  // [N]
  }

  // ケーブルの方向に張力を加える
  const auto W_axis = -W_Pos_PQ.Normalized();
  const auto W_force = T * W_axis;  // Qに働く力
  link_->AddWorldWrench(ecm, W_force, math::Vector3d::Zero, B_Pos_BQ_);
}

void GazeboTetherStationForcePlugin::getParamsCb(
  const GetSrv::Request::ConstSharedPtr&,
  const GetSrv::Response::SharedPtr& res)
{
  res->params = params_;
  return;
}

void GazeboTetherStationForcePlugin::setParamsCb(
  const SetSrv::Request::ConstSharedPtr& req,
  const SetSrv::Response::SharedPtr& res)
{
  res->params = params_;

  // Tension
  if (req->params.tension < 0)
  {
    gzerr << "Tension must be non-negative." << endl;
    res->success = false;
    return;
  }
  params_.tension = res->params.tension = req->params.tension;

  // Maximum length
  if (req->params.maximum_length <= 0)
  {
    gzerr << "Maximum length must be positive." << endl;
    res->success = false;
    return;
  }
  params_.maximum_length = res->params.maximum_length = req->params.maximum_length;

  res->success = true;
  return;
}
}  // namespace gazebo

GZ_ADD_PLUGIN(
  gazebo::GazeboTetherStationForcePlugin,
  sim::System,
  gazebo::GazeboTetherStationForcePlugin::ISystemConfigure,
  gazebo::GazeboTetherStationForcePlugin::ISystemPreUpdate)
