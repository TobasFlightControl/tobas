#include <gz/sim/Link.hh>

#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_tools/model_mass_holder.hpp>
#include <tobas_gazebo_tools/utils.hpp>
#include <tobas_std_tools/check.hpp>

#include <tobas_gazebo_msgs/srv/get_tether_params.hpp>
#include <tobas_gazebo_msgs/srv/set_tether_params.hpp>

#include "tobas_gazebo_system_plugins/common/common.hpp"
#include "tobas_gazebo_system_plugins/sdf.hpp"

namespace cmp = gz::sim::components;

namespace gazebo
{
class GazeboTetherStationForcePlugin : public BaseNode,
                                       public gz::sim::System,
                                       public gz::sim::ISystemConfigure,
                                       public gz::sim::ISystemPreUpdate
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
    const gz::sim::Entity& model_entity,
    const sdf::ElementConstPtr& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager&) override;

  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override;

private:
  // SDF parameters
  std::string link_name_;
  gz::math::Vector3d W_Pos_WP_;
  gz::math::Vector3d B_Pos_BQ_;
  double init_tension_;     // [N]
  double init_max_length_;  // [m]
  double young_;            // [MPa] ヤング率 (Young Modulus)
  double csa_;              // [mm^2] 断面積 (Cross-Sectional Area)

  tobas_gazebo_msgs::msg::TetherParams params_;

  std::shared_ptr<gz::sim::Link> link_;

  const cmp::WorldPose* pose_W_;
  const cmp::WorldLinearVelocity* linvel_W_;
  const cmp::WorldAngularVelocity* angvel_W_;

  ModelMassHolder mass_holder_;

  ros2::ServiceServerPtr<GetSrv> get_params_ss_;
  ros2::ServiceServerPtr<SetSrv> set_params_ss_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);

  void getParamsCb(const GetSrv::Request::ConstSharedPtr& req, const GetSrv::Response::SharedPtr& res);
  void setParamsCb(const SetSrv::Request::ConstSharedPtr& req, const SetSrv::Response::SharedPtr& res);
};

GazeboTetherStationForcePlugin::GazeboTetherStationForcePlugin()
{
}

void GazeboTetherStationForcePlugin::Configure(
  const gz::sim::Entity& model_entity,
  const sdf::ElementConstPtr& sdf,
  gz::sim::EntityComponentManager& ecm,
  gz::sim::EventManager&)
{
  initialize("gazebo_tether_station_force_plugin", sdf);
  getSdfParams(sdf);

  params_.tension = init_tension_;
  params_.maximum_length = init_max_length_;

  const auto link_entity = ecm.EntityByComponents(cmp::Link(), cmp::ParentEntity(model_entity), cmp::Name(link_name_));
  link_ = std::make_shared<gz::sim::Link>(link_entity);
  if (!link_->Valid(ecm)) {
    TOBAS_EXIT("Failed to find the specified link \"", link_name_, "\".");
  }

  TOBAS_CHECK(pose_W_ = getComponent<cmp::WorldPose>(link_entity, ecm));
  TOBAS_CHECK(linvel_W_ = getComponent<cmp::WorldLinearVelocity>(link_entity, ecm));
  TOBAS_CHECK(angvel_W_ = getComponent<cmp::WorldAngularVelocity>(link_entity, ecm));

  if (!mass_holder_.initialize(model_entity, ecm)) {
    TOBAS_EXIT("Failed to initialize model mass holder.");
  }

  get_params_ss_ = createService<GetSrv>(kGetTetherParamsSrv, &self::getParamsCb, this);
  set_params_ss_ = createService<SetSrv>(kSetTetherParamsSrv, &self::setParamsCb, this);
}

void GazeboTetherStationForcePlugin::PreUpdate(const gz::sim::UpdateInfo&, gz::sim::EntityComponentManager& ecm)
{
  // 現在の状態を取得
  const auto& T_W_B = pose_W_->Data();
  const auto& W_Pos_WB = T_W_B.Pos();
  const auto& R_W_B = T_W_B.Rot();
  const auto& linvel_W = linvel_W_->Data();
  const auto& angvel_W = angvel_W_->Data();

  // ケーブルの端点間ベクトルを計算
  const auto W_Pos_BQ = R_W_B.RotateVector(B_Pos_BQ_);
  const auto W_Pos_PQ = W_Pos_WB - W_Pos_WP_ + W_Pos_BQ;

  // ケーブルが伸び切っていない場合は一定張力
  auto T = params_.tension;  // [N]

  // ケーブル長が限界以上ならばワイヤロープの弾粘性モデル
  const auto length = W_Pos_PQ.Length();  // [m]
  if (length > params_.maximum_length) {
    // ケーブル長の変位を計算
    const auto x = length - params_.maximum_length;  // [m]

    // ケーブル長の変化率を計算
    const auto W_Vel_PQ = linvel_W + angvel_W.Cross(W_Pos_BQ);  // [m]
    const auto xd = W_Vel_PQ.Dot(W_Pos_PQ) / length;            // [m/s]

    // マスバネダンパ系の係数
    const auto m = mass_holder_.getMass();                  // [kg]
    const auto k = young_ * csa_ * params_.maximum_length;  // [N/m]
    const auto d = 2 * sqrt(m * k);                         // [Ns/m] 臨海減衰する粘性係数

    // ケーブルにかかる力を計算
    T = k * x + d * xd;  // [N]
  }

  // ケーブルの方向に張力を加える
  const auto axis_W = -W_Pos_PQ.Normalized();
  const auto force_W = T * axis_W;
  link_->AddWorldForce(ecm, force_W, B_Pos_BQ_);
}

void GazeboTetherStationForcePlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "worldEnd", W_Pos_WP_, gz::math::Vector3d::Zero);
  getSdfParam(sdf, "droneEnd", B_Pos_BQ_, gz::math::Vector3d::Zero);
  getSdfParam(sdf, "initialTension", init_tension_, kDefaultInitTension, kNonNegative);
  getSdfParam(sdf, "initialMaximumLength", init_max_length_, kDefaultInitMaxLength, kPositive);
  getSdfParam(sdf, "youngModulus", young_, kDefaultYoungModulus, kPositive);
  getSdfParam(sdf, "crossSectionArea", csa_, kDefaultCrossSectionArea, kPositive);
}

void GazeboTetherStationForcePlugin::getParamsCb(
  const GetSrv::Request::ConstSharedPtr&,
  const GetSrv::Response::SharedPtr& res)
{
  res->params = params_;
}

void GazeboTetherStationForcePlugin::setParamsCb(
  const SetSrv::Request::ConstSharedPtr& req,
  const SetSrv::Response::SharedPtr& res)
{
  res->params = params_;

  // Tension
  if (req->params.tension < 0) {
    TOBAS_ERROR("Tension must be non-negative.");
    res->success = false;
    return;
  }
  params_.tension = res->params.tension = req->params.tension;

  // Maximum length
  if (req->params.maximum_length <= 0) {
    TOBAS_ERROR(": Maximum length must be positive.");
    res->success = false;
    return;
  }
  params_.maximum_length = res->params.maximum_length = req->params.maximum_length;

  res->success = true;
}
}  // namespace gazebo

GZ_ADD_PLUGIN(
  gazebo::GazeboTetherStationForcePlugin,
  gz::sim::System,
  gazebo::GazeboTetherStationForcePlugin::ISystemConfigure,
  gazebo::GazeboTetherStationForcePlugin::ISystemPreUpdate)
