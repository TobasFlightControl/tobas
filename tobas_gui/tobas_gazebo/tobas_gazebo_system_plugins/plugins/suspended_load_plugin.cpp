#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/marker.pb.h>
#include <gz/sim/Link.hh>
#include <gz/sim/World.hh>
#include <gz/transport/Node.hh>

#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_conversions/gazebo_msg.hpp>
#include <tobas_gazebo_tools/model_mass_holder.hpp>
#include <tobas_gazebo_tools/utils.hpp>
#include <tobas_std_tools/check.hpp>

#include <tobas_gazebo_msgs/srv/attach_suspended_load.hpp>
#include <tobas_gazebo_msgs/srv/detach_suspended_load.hpp>

#include "tobas_gazebo_system_plugins/common/common.hpp"
#include "tobas_gazebo_system_plugins/rate_manager.hpp"
#include "tobas_gazebo_system_plugins/sdf.hpp"
#include "tobas_gazebo_system_plugins/sdf_string.hpp"
#include "tobas_gazebo_system_plugins/world.hpp"

namespace cmp = gz::sim::components;

namespace gazebo
{
class GazeboSuspendedLoadPlugin : public BaseNode,
                                  public gz::sim::System,
                                  public gz::sim::ISystemConfigure,
                                  public gz::sim::ISystemPreUpdate
{
  // Constants
  static constexpr char kPluginName[] = "gazebo_suspended_load_plugin";
  static constexpr int kUpdateMarkerRate = 60;  // [Hz]

  // Default parameters
  static constexpr double kDefaultYoungModulus = 200.;     // [MPa] 低密度ポリエチレン
  static constexpr double kDefaultCrossSectionArea = 50.;  // [mm^2]
  static constexpr uint64_t kDefaultLineId = 1;

  using self = GazeboSuspendedLoadPlugin;
  using AttachSrv = tobas_gazebo_msgs::srv::AttachSuspendedLoad;
  using DetachSrv = tobas_gazebo_msgs::srv::DetachSuspendedLoad;

public:
  explicit GazeboSuspendedLoadPlugin();

  void Configure(
    const gz::sim::Entity& model_entity,
    const sdf::ElementConstPtr& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager&) override;

  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override;

private:
  // SDF parameters
  std::string link_name_;
  gz::math::Vector3d B_Pos_BP_;
  double young_;  // [MPa] ヤング率 (Young Modulus)
  double csa_;    // [mm^2] 断面積 (Cross-Sectional Area)
  uint64_t line_id_;

  // Load parameters
  bool load_exist_ = false;
  double cable_length_;

  std::string world_name_;
  std::shared_ptr<gz::sim::Link> link_;

  const cmp::WorldPose* pose_W_;
  const cmp::WorldLinearVelocity* linvel_W_;
  const cmp::WorldAngularVelocity* angvel_W_;

  ModelMassHolder mass_holder_;
  RateManager rate_manager_;

  ros2::ServiceServerPtr<AttachSrv> attach_load_ss_;
  ros2::ServiceServerPtr<DetachSrv> detach_load_ss_;

  gz::transport::Node node_;
  gz::msgs::Marker marker_;
  gz::msgs::Vector3d* line_p0_;
  gz::msgs::Vector3d* line_p1_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);

  void attachLoadCb(const AttachSrv::Request::ConstSharedPtr& req, const AttachSrv::Response::SharedPtr& res);
  void detachLoadCb(const DetachSrv::Request::ConstSharedPtr& req, const DetachSrv::Response::SharedPtr& res);
};

GazeboSuspendedLoadPlugin::GazeboSuspendedLoadPlugin() : rate_manager_(kUpdateMarkerRate)
{
}

void GazeboSuspendedLoadPlugin::Configure(
  const gz::sim::Entity& model_entity,
  const sdf::ElementConstPtr& sdf,
  gz::sim::EntityComponentManager& ecm,
  gz::sim::EventManager&)
{
  initialize(kPluginName, sdf);
  getSdfParams(sdf);

  const auto world_name = getWorldName(ecm);
  if (!world_name) {
    TOBAS_EXIT("Failed to get the world name: ", world_name.error());
  }
  world_name_ = world_name.value();

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

  attach_load_ss_ = createService<AttachSrv>(kGetTetherParamsSrv, &self::attachLoadCb, this);
  detach_load_ss_ = createService<DetachSrv>(kSetTetherParamsSrv, &self::detachLoadCb, this);

  marker_.set_action(gz::msgs::Marker::ADD_MODIFY);
  marker_.set_ns(kPluginName);
  marker_.set_id(line_id_);
  marker_.set_type(gz::msgs::Marker::LINE_LIST);
  line_p0_ = marker_.add_point();
  line_p1_ = marker_.add_point();
}

void GazeboSuspendedLoadPlugin::PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm)
{
  if (!load_exist_) {
    return;
  }

  // 機体の状態を取得
  const auto& T_W_B = pose_W_->Data();
  const auto& W_Pos_WB = T_W_B.Pos();
  const auto& R_W_B = T_W_B.Rot();
  const auto& linvel_W = linvel_W_->Data();
  const auto& angvel_W = angvel_W_->Data();

  // 荷重の状態を取得
  gz::math::Vector3d W_Pos_WQ;  // TODO

  // ケーブルの端点間ベクトルを計算
  const auto W_Pos_BP = R_W_B.RotateVector(B_Pos_BP_);
  const auto W_Pos_WP = W_Pos_WB + W_Pos_BP;
  const auto W_Pos_PQ = W_Pos_WQ - W_Pos_WP;
  const auto length = W_Pos_PQ.Length();  // [m]

  // ケーブル長が限界以上ならばワイヤロープの弾粘性モデルに従って張力を加える
  if (length > cable_length_) {
    // ケーブル長の変位を計算
    const auto x = length - cable_length_;  // [m]

    // ケーブル長の変化率を計算
    const auto W_Vel_PQ = linvel_W + angvel_W.Cross(W_Pos_BP);  // [m]
    const auto xd = W_Vel_PQ.Dot(W_Pos_PQ) / length;            // [m/s]

    // マスバネダンパ系の係数
    const auto m = mass_holder_.getMass();         // [kg]
    const auto k = young_ * csa_ * cable_length_;  // [N/m]
    const auto d = 2 * sqrt(m * k);                // [Ns/m] 臨海減衰する粘性係数

    // ケーブルにかかる力を計算
    const auto T = k * x + d * xd;  // [N]

    // ケーブルの方向に張力を加える
    const auto axis_W = -W_Pos_PQ.Normalized();
    const auto force_W = T * axis_W;
    link_->AddWorldForce(ecm, force_W, B_Pos_BP_);
  }

  // 描画用のラインマーカを更新
  if (rate_manager_.update(info.simTime)) {
    vector3dGzToMsg(W_Pos_WP, *line_p0_);
    vector3dGzToMsg(W_Pos_WQ, *line_p1_);
    node_.Request(kGzMarkerSrv, marker_);
  }
}

void GazeboSuspendedLoadPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "droneEnd", B_Pos_BP_, gz::math::Vector3d::Zero);
  getSdfParam(sdf, "youngModulus", young_, kDefaultYoungModulus, kPositive);
  getSdfParam(sdf, "crossSectionArea", csa_, kDefaultCrossSectionArea, kPositive);
  getSdfParam(sdf, "lineId", line_id_, kDefaultLineId, kPositive);  // 0だとIDがランダムに割り当てられて無限に増える
}

void GazeboSuspendedLoadPlugin::attachLoadCb(
  const AttachSrv::Request::ConstSharedPtr& req,
  const AttachSrv::Response::SharedPtr& res)
{
  if (req->load_size <= 0.) {
    res->success = false;
    res->message = "Load size must be positive.";
    return;
  }
  if (req->load_mass <= 0.) {
    res->success = false;
    res->message = "Load mass must be positive.";
    return;
  }
  if (req->cable_length <= 0.) {
    res->success = false;
    res->message = "Cable length must be positive.";
    return;
  }

  gz::msgs::EntityFactory gzreq;
  gzreq.set_sdf(makeBoxSdf("load", req->load_size, req->load_size, req->load_size, req->load_mass, 0., 0., 0.));
  gzreq.set_allow_renaming(true);

  gz::msgs::Boolean gzrep;
  bool gzres = false;

  const auto gzsrv_name = "/world/" + world_name_ + "/create";
  if (!node_.Request(gzsrv_name, gzreq, 1000, gzrep, gzres)) {
    res->success = false;
    res->message = "Gazebo service call timeout: " + gzsrv_name;
    return;
  }
  if (!gzres) {
    res->success = false;
    res->message = "Gazebo service call failed.";
    return;
  }
  if (!gzrep.data()) {
    res->success = false;
    res->message = "Failed to create the load entity.";
    return;
  }

  cable_length_ = req->cable_length;
  load_exist_ = true;

  res->success = true;
  res->message.clear();
}

void GazeboSuspendedLoadPlugin::detachLoadCb(
  const DetachSrv::Request::ConstSharedPtr&,
  const DetachSrv::Response::SharedPtr& res)
{
  load_exist_ = false;

  res->success = true;
  res->message.clear();
}
}  // namespace gazebo

GZ_ADD_PLUGIN(
  gazebo::GazeboSuspendedLoadPlugin,
  gz::sim::System,
  gazebo::GazeboSuspendedLoadPlugin::ISystemConfigure,
  gazebo::GazeboSuspendedLoadPlugin::ISystemPreUpdate)
