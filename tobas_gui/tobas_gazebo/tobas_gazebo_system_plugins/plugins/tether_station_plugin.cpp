#include <gz/msgs/marker.pb.h>
#include <gz/sim/Link.hh>
#include <gz/transport/Node.hh>

#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_conversions/gazebo_msg.hpp>
#include <tobas_gazebo_tools/model_mass_holder.hpp>
#include <tobas_gazebo_tools/utils.hpp>
#include <tobas_std_tools/check.hpp>

#include <tobas_gazebo_msgs/srv/get_tether_params.hpp>
#include <tobas_gazebo_msgs/srv/set_tether_params.hpp>

#include "tobas_gazebo_system_plugins/common/common.hpp"
#include "tobas_gazebo_system_plugins/rate_manager.hpp"
#include "tobas_gazebo_system_plugins/sdf.hpp"

namespace cmp = gz::sim::components;

namespace tobas
{
namespace gazebo
{
class GazeboTetherStationPlugin : public BaseNode,
                                  public gz::sim::System,
                                  public gz::sim::ISystemConfigure,
                                  public gz::sim::ISystemPreUpdate
{
  // Constants
  static constexpr char kPluginName[] = "gazebo_tether_station_plugin";
  static constexpr int kUpdateMarkerRate = 60;  // [Hz]

  // Default parameters
  static constexpr double kDefaultInitTension = 1.;       // [N]
  static constexpr double kDefaultInitMaxLength = 5.;     // [N]
  static constexpr double kDefaultYoungModulus = 200.;    // [MPa] 低密度ポリエチレン
  static constexpr double kDefaultCrossSectionArea = 1.;  // [mm^2]

  using self = GazeboTetherStationPlugin;
  using GetSrv = tobas_gazebo_msgs::srv::GetTetherParams;
  using SetSrv = tobas_gazebo_msgs::srv::SetTetherParams;

public:
  explicit GazeboTetherStationPlugin();

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
  RateManager rate_manager_;

  ros2::ServiceServerPtr<GetSrv> get_params_ss_;
  ros2::ServiceServerPtr<SetSrv> set_params_ss_;

  gz::transport::Node node_;
  gz::msgs::Marker marker_;
  gz::msgs::Vector3d* line_p0_;
  gz::msgs::Vector3d* line_p1_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);

  void getParamsCb(const GetSrv::Request::ConstSharedPtr& req, const GetSrv::Response::SharedPtr& res);
  void setParamsCb(const SetSrv::Request::ConstSharedPtr& req, const SetSrv::Response::SharedPtr& res);
};

GazeboTetherStationPlugin::GazeboTetherStationPlugin() : rate_manager_(kUpdateMarkerRate)
{
}

void GazeboTetherStationPlugin::Configure(
  const gz::sim::Entity& model_entity,
  const sdf::ElementConstPtr& sdf,
  gz::sim::EntityComponentManager& ecm,
  gz::sim::EventManager&)
{
  initialize(kPluginName, sdf);
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

  marker_.set_action(gz::msgs::Marker::ADD_MODIFY);
  marker_.set_ns(kPluginName);
  marker_.set_id(1);  // 0だとIDがランダムに割り当てられて無限に増えてしまう
  marker_.set_type(gz::msgs::Marker::LINE_LIST);
  line_p0_ = marker_.add_point();
  line_p1_ = marker_.add_point();
}

void GazeboTetherStationPlugin::PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm)
{
  // 現在の状態を取得
  const auto& T_W_B = pose_W_->Data();
  const auto& W_Pos_WB = T_W_B.Pos();
  const auto& R_W_B = T_W_B.Rot();
  const auto& linvel_W = linvel_W_->Data();
  const auto& angvel_W = angvel_W_->Data();

  // ケーブルの端点間ベクトルを計算
  const auto W_Pos_BQ = R_W_B.RotateVector(B_Pos_BQ_);
  const auto W_Pos_WQ = W_Pos_WB + W_Pos_BQ;
  const auto W_Pos_PQ = W_Pos_WQ - W_Pos_WP_;
  const auto length = W_Pos_PQ.Length();  // [m]

  // 張力を決定
  double T;                               // [N]
  if (length > params_.maximum_length) {  // ケーブル長が限界以上ならばワイヤロープの弾粘性モデル
    // ケーブル長の変位を計算
    const auto x = length - params_.maximum_length;  // [m]

    // ケーブル長の変化率を計算
    const auto W_Vel_PQ = linvel_W + angvel_W.Cross(W_Pos_BQ);  // W_Vel_PQ = W_Vel_WQ - W_Vel_WP = W_Vel_WQ
    const auto xd = W_Vel_PQ.Dot(W_Pos_PQ) / length;  // 相対速度をケーブル方向に射影したものがケーブル長の変化率

    // マスバネダンパ系の係数
    const auto m = mass_holder_.getMass();                  // [kg]
    const auto k = young_ * csa_ * params_.maximum_length;  // [N/m]
    const auto d = 2 * sqrt(m * k);                         // [Ns/m] 臨海減衰する粘性係数

    // ケーブルにかかる力を計算
    T = k * x + d * xd;
  }
  else {  // ケーブルが伸び切っていない場合は一定張力
    T = params_.tension;
  }

  // ケーブルの方向に張力を加える
  const auto axis_W = -W_Pos_PQ.Normalized();
  const auto force_W = T * axis_W;
  link_->AddWorldForce(ecm, force_W, B_Pos_BQ_);

  // 描画用のラインマーカを更新
  if (rate_manager_.update(info.simTime)) {
    vector3dGzToMsg(W_Pos_WP_, *line_p0_);
    vector3dGzToMsg(W_Pos_WQ, *line_p1_);
    node_.Request(kGzMarkerSrv, marker_);
  }
}

void GazeboTetherStationPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "worldEnd", W_Pos_WP_, gz::math::Vector3d::Zero);
  getSdfParam(sdf, "droneEnd", B_Pos_BQ_, gz::math::Vector3d::Zero);
  getSdfParam(sdf, "initialTension", init_tension_, kDefaultInitTension, kNonNegative);
  getSdfParam(sdf, "initialMaximumLength", init_max_length_, kDefaultInitMaxLength, kPositive);
  getSdfParam(sdf, "youngModulus", young_, kDefaultYoungModulus, kPositive);
  getSdfParam(sdf, "crossSectionArea", csa_, kDefaultCrossSectionArea, kPositive);
}

void GazeboTetherStationPlugin::getParamsCb(
  const GetSrv::Request::ConstSharedPtr&,
  const GetSrv::Response::SharedPtr& res)
{
  res->params = params_;
}

void GazeboTetherStationPlugin::setParamsCb(
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
}  // namespace tobas

GZ_ADD_PLUGIN(
  tobas::gazebo::GazeboTetherStationPlugin,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)
