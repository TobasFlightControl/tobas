// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/marker.pb.h>
#include <gz/sim/Link.hh>
#include <gz/transport/Node.hh>

#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_conversions/gazebo_msg.hpp>
#include <tobas_gazebo_conversions/gazebo_ros.hpp>
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

namespace tobas
{
namespace gazebo
{
class GazeboSuspendedLoadPlugin : public BaseNode,
                                  public gz::sim::System,
                                  public gz::sim::ISystemConfigure,
                                  public gz::sim::ISystemPreUpdate
{
  static constexpr char kPluginName[] = "gazebo_suspended_load_plugin";
  static constexpr char kLoadNamePrefix[] = "load_";
  static constexpr double kStopLoadRotationTimeConst = 10.0;  // [s]
  static constexpr int kUpdateMarkerRate = 60;                // [Hz]

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

  // Aircraft
  std::shared_ptr<gz::sim::Link> base_link_;
  const cmp::WorldPose* W_Pose_B_ = nullptr;
  const cmp::WorldLinearVelocity* W_Vel_WB_ = nullptr;
  const cmp::WorldAngularVelocity* W_Gyro_WB_ = nullptr;

  // Load
  gz::math::Vector3d B_Pos_BP_;
  gz::math::Vector3d L_Pos_LQ_;
  double load_mass_;
  gz::math::Matrix3d load_inertia_ = gz::math::Matrix3d::Zero;
  double cable_length_;  // [m] Natural cable length
  double cable_young_;   // [Pa] Young modulus.
  double cable_csa_;     // [m^2] Cross-sectional area.
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

  // Keep SDF parameters minimal so values can be adjusted from the GUI.
  getSdfParam(sdf, "linkName", link_name_);

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
  marker_.set_id(1);  // If this is 0, IDs are assigned randomly and increase without bound.
  marker_.set_type(gz::msgs::Marker::LINE_LIST);
  line_p0_ = marker_.add_point();
  line_p1_ = marker_.add_point();
}

void GazeboSuspendedLoadPlugin::PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm)
{
  if (!load_exist_) {
    return;
  }

  // Make the load state accessible.
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

    const auto link_entity = load_model.CanonicalLink(ecm);  // Get the canonical link of the model.
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

    return;  // Do not apply force in the cycle where components were obtained because values are not correct yet.
  }

  // Get the vehicle state.
  const auto& W_Pose_B = W_Pose_B_->Data();
  const auto& W_Pos_WB = W_Pose_B.Pos();
  const auto& W_Rot_B = W_Pose_B.Rot();
  const auto& W_Vel_WB = W_Vel_WB_->Data();
  const auto& W_Gyro_WB = W_Gyro_WB_->Data();

  // Get the load state.
  const auto& W_Pose_L = W_Pose_L_->Data();
  const auto& W_Pos_WL = W_Pose_L.Pos();
  const auto& W_Rot_L = W_Pose_L.Rot();
  const auto& W_Vel_WL = W_Vel_WL_->Data();
  const auto& W_Gyro_WL = W_Gyro_WL_->Data();

  // Compute the vector between cable endpoints.
  const auto W_Pos_BP = W_Rot_B.RotateVector(B_Pos_BP_);
  const auto W_Pos_WP = W_Pos_WB + W_Pos_BP;
  const auto W_Pos_LQ = W_Rot_L.RotateVector(L_Pos_LQ_);
  const auto W_Pos_WQ = W_Pos_WL + W_Pos_LQ;
  const auto W_Pos_PQ = W_Pos_WQ - W_Pos_WP;
  const auto length = W_Pos_PQ.Length();  // [m]

  // Apply tension according to a wire-rope elastic-viscous model if the cable length exceeds the limit.
  if (length > cable_length_) {
    // Compute cable-length displacement.
    const auto x = length - cable_length_;  // [m]

    // Compute cable-length rate.
    const auto W_Vel_WP = W_Vel_WB + W_Gyro_WB.Cross(W_Pos_BP);
    const auto W_Vel_WQ = W_Vel_WL + W_Gyro_WL.Cross(W_Pos_LQ);
    const auto W_Vel_PQ = W_Vel_WQ - W_Vel_WP;
    const auto xd = W_Vel_PQ.Dot(W_Pos_PQ) / length;

    // Mass-spring-damper coefficients.
    const auto& m1 = mass_holder_.getMass();
    const auto& m2 = load_mass_;
    const auto m = m1 * m2 / (m1 + m2);                        // [kg] Equivalent mass of relative motion (memo: 3-47).
    const auto k = cable_young_ * cable_csa_ * cable_length_;  // [N/m]
    const auto d = 2 * std::sqrt(m * k);                       // [Ns/m] Viscous coefficient for critical damping.

    // Compute force applied to the cable.
    const auto T = k * x + d * xd;  // [N]

    // Torque that counteracts load rotation, simulating air resistance and friction at the cable joint.
    const auto L_Gyro_WL = W_Rot_L.RotateVectorReverse(W_Gyro_WL);
    const auto L_DGyro_WL = -(1.0 / kStopLoadRotationTimeConst) * L_Gyro_WL;
    const auto L_Torque_WL = load_inertia_ * L_DGyro_WL + L_Gyro_WL.Cross(load_inertia_ * L_Gyro_WL);
    const auto W_Torque_WL = W_Rot_L.RotateVector(L_Torque_WL);

    // Apply tension along the cable direction and anti-rotation torque.
    const auto W_Force_PQ = T * W_Pos_PQ.Normalized();
    base_link_->AddWorldForce(ecm, W_Force_PQ, B_Pos_BP_);
    load_link_->AddWorldWrench(ecm, -W_Force_PQ, W_Torque_WL, L_Pos_LQ_);
  }

  // Update the line marker for visualization.
  if (rate_manager_.update(info.simTime)) {
    vector3dGzToMsg(W_Pos_WP, *line_p0_);
    vector3dGzToMsg(W_Pos_WQ, *line_p1_);
    marker_.set_action(gz::msgs::Marker::ADD_MODIFY);
    if (!node_.Request(kGzMarkerSrv, marker_)) {
      TOBAS_ERROR("Failed to update the line marker.");
    }
  }
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

  if (req->load_sx <= 0.0 || req->load_sy <= 0.0 || req->load_sz <= 0.0) {
    res->success = false;
    res->message = "Load size must be positive.";
    return;
  }
  if (req->load_mass <= 0.0) {
    res->success = false;
    res->message = "Load mass must be positive.";
    return;
  }
  if (req->cable_length <= 0.0) {
    res->success = false;
    res->message = "Cable length must be positive.";
    return;
  }
  if (req->cable_young_modulus <= 0.0) {
    res->success = false;
    res->message = "Cable young modulus must be positive.";
    return;
  }
  if (req->cable_cross_sectional_area <= 0.0) {
    res->success = false;
    res->message = "Cable cross section area must be positive.";
    return;
  }

  const auto sz_2 = req->load_sz / 2;

  // Increment the index to avoid duplicate model names.
  ++load_index_;

  // Determine spawn position.
  const auto& W_Pose_B = W_Pose_B_->Data();
  const auto& W_Pos_WB = W_Pose_B.Pos();
  const auto& W_Rot_B = W_Pose_B.Rot();
  const auto W_Pos_WP = W_Pos_WB + W_Rot_B.RotateVector(B_Pos_BP_);  // Attachment position.
  const auto px = W_Pos_WP.X();
  const auto py = W_Pos_WP.Y();
  const auto pz = std::max(
    W_Pos_WP.Z() - req->cable_length - sz_2, sz_2);  // Lower by the cable length, but keep it above the ground.

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

  vectorRosToGazebo(req->attachment_point, B_Pos_BP_);
  L_Pos_LQ_.Set(0.0, 0.0, sz_2);  // Assume the cable is attached to the center of the cuboid top face.
  load_mass_ = req->load_mass;
  cable_length_ = req->cable_length;
  cable_young_ = req->cable_young_modulus;
  cable_csa_ = req->cable_cross_sectional_area;

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
}  // namespace tobas

GZ_ADD_PLUGIN(
  tobas::gazebo::GazeboSuspendedLoadPlugin,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)
