#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/marker.pb.h>
#include <gz/sim/Link.hh>
#include <gz/sim/World.hh>
#include <gz/transport/Node.hh>

#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_conversions/gazebo_msg.hpp>
#include <tobas_gazebo_tools/model_mass_holder.hpp>
#include <tobas_gazebo_tools/utils.hpp>

#include <tobas_gazebo_msgs/srv/attach_suspended_load.hpp>
#include <tobas_gazebo_msgs/srv/detach_suspended_load.hpp>

#include "tobas_gazebo_system_plugins/common/common.hpp"
#include "tobas_gazebo_system_plugins/inertia.hpp"
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
  static constexpr char kLoadNamePrefix[] = "load_";
  static constexpr double kStopLoadRotationTimeConst = 10.;  // [s]
  static constexpr int kUpdateMarkerRate = 60;               // [Hz]

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
    const gz::sim::Entity& model,
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

  // Aircraft
  std::shared_ptr<gz::sim::Link> base_link_;
  const cmp::WorldPose* W_Pose_B_ = nullptr;
  const cmp::WorldLinearVelocity* W_Vel_WB_ = nullptr;
  const cmp::WorldAngularVelocity* W_Gyro_WB_ = nullptr;

  // Load
  gz::math::Vector3d L_Pos_LQ_;
  double load_mass_;
  gz::math::Matrix3d load_inertia_ = gz::math::Matrix3d::Zero;
  double cable_length_;
  bool load_exist_ = false;
  int load_index_ = 0;
  std::shared_ptr<gz::sim::Link> load_link_;
  const cmp::WorldPose* W_Pose_L_ = nullptr;
  const cmp::WorldLinearVelocity* W_Vel_WL_ = nullptr;
  const cmp::WorldAngularVelocity* W_Gyro_WL_ = nullptr;

  std::string world_name_;
  ModelMassHolder mass_holder_;
  RateManager rate_manager_;

  ros2::ServiceServerPtr<AttachSrv> attach_load_ss_;
  ros2::ServiceServerPtr<DetachSrv> detach_load_ss_;

  gz::transport::Node node_;
  gz::msgs::Marker marker_;
  gz::msgs::Vector3d* line_p0_;
  gz::msgs::Vector3d* line_p1_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
  std::string loadName() const;

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
  base_link_ = std::make_shared<gz::sim::Link>(link_entity);
  if (!base_link_->Valid(ecm)) {
    TOBAS_EXIT("Failed to find the specified link \"", link_name_, "\".");
  }

  if (!(W_Pose_B_ = getComponent<cmp::WorldPose>(link_entity, ecm))) {
    TOBAS_EXIT("Failed to get the world pose of \"", link_name_, "\".");
  }
  if (!(W_Vel_WB_ = getComponent<cmp::WorldLinearVelocity>(link_entity, ecm))) {
    TOBAS_EXIT("Failed to get the world linear velocity of \"", link_name_, "\".");
  }
  if (!(W_Gyro_WB_ = getComponent<cmp::WorldAngularVelocity>(link_entity, ecm))) {
    TOBAS_EXIT("Failed to get the world angular velocity of \"", link_name_, "\".");
  }

  if (!mass_holder_.initialize(model_entity, ecm)) {
    TOBAS_EXIT("Failed to initialize model mass holder.");
  }

  attach_load_ss_ = createService<AttachSrv>(kAttachSuspenedLoadSrv, &self::attachLoadCb, this);
  detach_load_ss_ = createService<DetachSrv>(kDetachSuspenedLoadSrv, &self::detachLoadCb, this);

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

  // 荷重の状態にアクセスできるようにする
  if (!load_link_) {
    const auto model_entity = ecm.EntityByComponents(cmp::Model(), cmp::Name(loadName()));
    if (model_entity == gz::sim::kNullEntity) {
      return;
    }
    TOBAS_INFO("Load entity is found: ", model_entity);

    const gz::sim::Model load_model(model_entity);
    if (!load_model.Valid(ecm)) {
      TOBAS_ERROR("Failed to find the load model.");
      load_exist_ = false;
      return;
    }

    const auto link_entity = load_model.CanonicalLink(ecm);  // モデルの代表リンクを取得
    load_link_ = std::make_shared<gz::sim::Link>(link_entity);
    if (!load_link_->Valid(ecm)) {
      TOBAS_EXIT("Failed to find the canonical link of the load.");
    }

    if (!(W_Pose_L_ = getComponent<cmp::WorldPose>(link_entity, ecm))) {
      TOBAS_ERROR("Failed to get the world pose of the load.");
      load_exist_ = false;
      return;
    }
    if (!(W_Vel_WL_ = getComponent<cmp::WorldLinearVelocity>(link_entity, ecm))) {
      TOBAS_ERROR("Failed to get the world linear velocity of the load.");
      load_exist_ = false;
      return;
    }
    if (!(W_Gyro_WL_ = getComponent<cmp::WorldAngularVelocity>(link_entity, ecm))) {
      TOBAS_ERROR("Failed to get the world angular velocity of the load.");
      load_exist_ = false;
      return;
    }

    return;  // コンポーネントを取得したサイクルは正しい値が出ないため力を加えない
  }

  // 機体の状態を取得
  const auto& W_Pose_B = W_Pose_B_->Data();
  const auto& W_Pos_WB = W_Pose_B.Pos();
  const auto& W_Rot_B = W_Pose_B.Rot();
  const auto& W_Vel_WB = W_Vel_WB_->Data();
  const auto& W_Gyro_WB = W_Gyro_WB_->Data();

  // 荷重の状態を取得
  const auto& W_Pose_L = W_Pose_L_->Data();
  const auto& W_Pos_WL = W_Pose_L.Pos();
  const auto& W_Rot_L = W_Pose_L.Rot();
  const auto& W_Vel_WL = W_Vel_WL_->Data();
  const auto& W_Gyro_WL = W_Gyro_WL_->Data();

  // ケーブルの端点間ベクトルを計算
  const auto W_Pos_BP = W_Rot_B.RotateVector(B_Pos_BP_);
  const auto W_Pos_WP = W_Pos_WB + W_Pos_BP;
  const auto W_Pos_LQ = W_Rot_L.RotateVector(L_Pos_LQ_);
  const auto W_Pos_WQ = W_Pos_WL + W_Pos_LQ;
  const auto W_Pos_PQ = W_Pos_WQ - W_Pos_WP;
  const auto length = W_Pos_PQ.Length();  // [m]

  // ケーブル長が限界以上ならばワイヤロープの弾粘性モデルに従って張力を加える
  if (length > cable_length_) {
    // ケーブル長の変位を計算
    const auto x = length - cable_length_;  // [m]

    // ケーブル長の変化率を計算
    const auto W_Vel_WP = W_Vel_WB + W_Gyro_WB.Cross(W_Pos_BP);
    const auto W_Vel_WQ = W_Vel_WL + W_Gyro_WL.Cross(W_Pos_LQ);
    const auto W_Vel_PQ = W_Vel_WQ - W_Vel_WP;
    const auto xd = W_Vel_PQ.Dot(W_Pos_PQ) / length;  // 相対速度をケーブル方向に射影したものがケーブル長の変化率

    // マスバネダンパ系の係数
    const auto& m1 = mass_holder_.getMass();
    const auto& m2 = load_mass_;
    const auto m = m1 * m2 / (m1 + m2);            // [kg] 相対運動の等価質量 (memo: 3-47)
    const auto k = young_ * csa_ * cable_length_;  // [N/m]
    const auto d = 2 * sqrt(m * k);                // [Ns/m] 臨海減衰する粘性係数

    // ケーブルにかかる力を計算
    const auto T = k * x + d * xd;  // [N]

    // 荷重の回転を打ち消す方向に働くトルク (空気抵抗やケーブル接続部の摩擦を模擬)
    const auto L_Gyro_WL = W_Rot_L.RotateVectorReverse(W_Gyro_WL);
    const auto L_DGyro_WL = -(1. / kStopLoadRotationTimeConst) * L_Gyro_WL;
    const auto L_Torque_WL = load_inertia_ * L_DGyro_WL + L_Gyro_WL.Cross(load_inertia_ * L_Gyro_WL);
    const auto W_Torque_WL = W_Rot_L.RotateVector(L_Torque_WL);

    // ケーブルの方向に張力を加える
    const auto W_Force_PQ = T * W_Pos_PQ.Normalized();
    base_link_->AddWorldForce(ecm, W_Force_PQ, B_Pos_BP_);
    load_link_->AddWorldWrench(ecm, -W_Force_PQ, W_Torque_WL, L_Pos_LQ_);
  }

  // 描画用のラインマーカを更新
  if (rate_manager_.update(info.simTime)) {
    vector3dGzToMsg(W_Pos_WP, *line_p0_);
    vector3dGzToMsg(W_Pos_WQ, *line_p1_);
    marker_.set_action(gz::msgs::Marker::ADD_MODIFY);
    if (!node_.Request(kGzMarkerSrv, marker_)) {
      TOBAS_ERROR("Failed to update the line marker.");
    }
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

std::string GazeboSuspendedLoadPlugin::loadName() const
{
  return kLoadNamePrefix + std::to_string(load_index_);
}

void GazeboSuspendedLoadPlugin::attachLoadCb(
  const AttachSrv::Request::ConstSharedPtr& req,
  const AttachSrv::Response::SharedPtr& res)
{
  if (load_exist_) {
    res->success = false;
    res->message = "Load is already attached.";
    return;
  }

  if (req->load_sx <= 0. || req->load_sy <= 0. || req->load_sz <= 0.) {
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

  const auto sz_2 = req->load_sz / 2;

  // モデル名が被らないようにインデックスを上げる
  ++load_index_;

  // 出現位置を決める
  const auto& W_Pose_B = W_Pose_B_->Data();
  const auto& W_Pos_WB = W_Pose_B.Pos();
  const auto& W_Rot_B = W_Pose_B.Rot();
  const auto W_Pos_WP = W_Pos_WB + W_Rot_B.RotateVector(B_Pos_BP_);  // 取り付け位置
  const auto px = W_Pos_WP.X();
  const auto py = W_Pos_WP.Y();
  const auto pz = std::max(W_Pos_WP.Z() - req->cable_length - sz_2, sz_2);  // ケーブルの長さ分だけ下げるが地面よりは上

  gz::msgs::EntityFactory gzreq;
  gzreq.set_sdf(makeBoxSdf(loadName(), req->load_sx, req->load_sy, req->load_sz, req->load_mass, px, py, pz));
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

  L_Pos_LQ_.Set(0., 0., sz_2);  // 直方体の上面の中央にケーブルを取り付ける想定
  load_mass_ = req->load_mass;
  cable_length_ = req->cable_length;

  const auto [ixx, iyy, izz] = boxInertia(req->load_sx, req->load_sy, req->load_sz, req->load_mass);
  load_inertia_.Set(0, 0, ixx);
  load_inertia_.Set(1, 1, iyy);
  load_inertia_.Set(2, 2, izz);

  load_exist_ = true;

  res->success = true;
  res->message.clear();
}

void GazeboSuspendedLoadPlugin::detachLoadCb(
  const DetachSrv::Request::ConstSharedPtr&,
  const DetachSrv::Response::SharedPtr& res)
{
  marker_.set_action(gz::msgs::Marker::DELETE_MARKER);
  if (!node_.Request(kGzMarkerSrv, marker_)) {
    res->success = false;
    res->message = "Failed to delete the line marker.";
    return;
  }

  load_exist_ = false;
  load_link_.reset();
  W_Pose_L_ = nullptr;
  W_Vel_WL_ = nullptr;
  W_Gyro_WL_ = nullptr;

  res->success = true;
  res->message.clear();
}
}  // namespace gazebo

GZ_ADD_PLUGIN(
  gazebo::GazeboSuspendedLoadPlugin,
  gz::sim::System,
  gazebo::GazeboSuspendedLoadPlugin::ISystemConfigure,
  gazebo::GazeboSuspendedLoadPlugin::ISystemPreUpdate)
