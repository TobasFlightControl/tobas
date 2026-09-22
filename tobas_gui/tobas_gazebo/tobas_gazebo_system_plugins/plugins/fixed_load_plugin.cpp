// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <condition_variable>
#include <mutex>
#include <optional>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>

#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_conversions/gazebo_ros.hpp>

#include <tobas_gazebo_msgs/srv/attach_fixed_load.hpp>
#include <tobas_gazebo_msgs/srv/detach_fixed_load.hpp>

#include "tobas_gazebo_system_plugins/common/common.hpp"
#include "tobas_gazebo_system_plugins/inertia.hpp"
#include "tobas_gazebo_system_plugins/sdf_string.hpp"

using namespace std::chrono_literals;
namespace cmp = gz::sim::components;

namespace tobas
{
namespace gazebo
{
class GazeboFixedLoadPlugin : public BaseNode,
                              public gz::sim::System,
                              public gz::sim::ISystemConfigure,
                              public gz::sim::ISystemPreUpdate
{
  using self = GazeboFixedLoadPlugin;
  using AttachSrv = tobas_gazebo_msgs::srv::AttachFixedLoad;
  using DetachSrv = tobas_gazebo_msgs::srv::DetachFixedLoad;

  struct Operation
  {
    // Request
    std::optional<AttachSrv::Request> attach;  // nullopt indicates a detach request.

    // Response
    bool success = false;
    std::string message;
  };

public:
  void Configure(
    const gz::sim::Entity& model,
    const sdf::ElementConstPtr& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager& events) override;

  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override;

private:
  std::optional<gz::sim::SdfEntityCreator> creator_;
  gz::sim::Entity world_entity_;
  gz::sim::Link base_link_;
  gz::sim::Entity joint_entity_;

  std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<Operation> pending_;
  bool done_ = false;
  int load_index_ = 0;

  ros2::ServiceServerPtr<AttachSrv> attach_load_ss_;
  ros2::ServiceServerPtr<DetachSrv> detach_load_ss_;

  void submit(Operation& operation);
  bool attachLoad(const AttachSrv::Request& req, std::string& message, gz::sim::EntityComponentManager& ecm);
  bool detachLoad(std::string& message, gz::sim::EntityComponentManager& ecm);

  void attachLoadCb(const AttachSrv::Request::ConstSharedPtr& req, const AttachSrv::Response::SharedPtr& res);
  void detachLoadCb(const DetachSrv::Request::ConstSharedPtr& req, const DetachSrv::Response::SharedPtr& res);
};

void GazeboFixedLoadPlugin::Configure(
  const gz::sim::Entity& model_entity,
  const sdf::ElementConstPtr& sdf,
  gz::sim::EntityComponentManager& ecm,
  gz::sim::EventManager& events)
{
  initialize("gazebo_fixed_load_plugin", sdf);

  world_entity_ = ecm.EntityByComponents(cmp::World());
  if (world_entity_ == gz::sim::kNullEntity) {
    TOBAS_EXIT("Failed to find the world entity.");
  }

  const gz::sim::Model model(model_entity);
  if (!model.Valid(ecm)) {
    TOBAS_EXIT("Failed to find model.");
  }

  const auto link_name = getSdfParam<std::string>(sdf, "linkName");
  base_link_ = gz::sim::Link(model.LinkByName(ecm, link_name));
  if (!base_link_.Valid(ecm)) {
    TOBAS_EXIT("Failed to find the specified link '", link_name, "'.");
  }

  creator_.emplace(ecm, events);

  attach_load_ss_ = createService<AttachSrv>(kAttachFixedLoadSrv, &self::attachLoadCb, this);
  detach_load_ss_ = createService<DetachSrv>(kDetachFixedLoadSrv, &self::detachLoadCb, this);
}

void GazeboFixedLoadPlugin::PreUpdate(const gz::sim::UpdateInfo&, gz::sim::EntityComponentManager& ecm)
{
  std::lock_guard lock(mutex_);

  if (!pending_) {
    return;
  }

  if (pending_->attach) {
    pending_->success = attachLoad(*pending_->attach, pending_->message, ecm);
  }
  else {
    pending_->success = detachLoad(pending_->message, ecm);
  }

  done_ = true;
  cv_.notify_all();
}

void GazeboFixedLoadPlugin::submit(Operation& operation)
{
  std::unique_lock lock(mutex_);

  if (pending_) {
    operation.success = false;
    operation.message = "Another request is pending.";
    return;
  }

  pending_ = operation;
  done_ = false;
  cv_.wait(lock, [this]() { return done_; });

  operation = *pending_;
  pending_.reset();
}

bool GazeboFixedLoadPlugin::attachLoad(
  const AttachSrv::Request& req,
  std::string& message,
  gz::sim::EntityComponentManager& ecm)
{
  if (joint_entity_ != gz::sim::kNullEntity && ecm.HasEntity(joint_entity_)) {
    message = "A fixed load is already attached.";
    return false;
  }

  if (req.load_size.x <= 0.0 || req.load_size.y <= 0.0 || req.load_size.z <= 0.0) {
    message = "Load dimensions must be positive.";
    return false;
  }
  if (req.load_mass <= 0.0) {
    message = "Load mass must be positive.";
    return false;
  }

  // Compute the load pose in the world frame.
  gz::math::Pose3d T_B_L;
  gazebo::poseRosToGazebo(req.load_pose, T_B_L);
  const auto T_W_B = gz::sim::worldPose(base_link_.Entity(), ecm);
  const auto T_W_L = T_W_B * T_B_L;

  // Include the attachment link entity to distinguish loads from different aircraft.
  const auto load_name = "fixed_load_" + std::to_string(base_link_.Entity()) + "_" + std::to_string(load_index_++);

  // Load the SDF.
  sdf::Root root;
  const auto sdf = makeBoxSdf(load_name, req.load_size.x, req.load_size.y, req.load_size.z, req.load_mass);
  const auto errors = root.LoadSdfString(sdf);
  if (!errors.empty() || !root.Model()) {
    message = "Failed to construct the load SDF.";
    return false;
  }

  // Create the load entity.
  const auto load_entity = creator_->CreateEntities(root.Model());
  if (load_entity == gz::sim::kNullEntity) {
    message = "Failed to create the load model.";
    return false;
  }
  creator_->SetParent(load_entity, world_entity_);

  // Set the ECM pose before physics sees the new model.
  // Changing the parsed SDF's raw pose alone does not update its already-resolved semantic pose graph.
  ecm.SetComponentData<cmp::Pose>(load_entity, T_W_L);
  const gz::sim::Link load_link(gz::sim::Model(load_entity).CanonicalLink(ecm));
  if (!load_link.Valid(ecm)) {
    creator_->RequestRemoveEntity(load_entity);
    message = "Failed to create the load link.";
    return false;
  }

  // The fixed joint makes the load follow the parent's rigid-body velocity,
  // including rotation about the offset. Do not leave velocity command components:
  // Gazebo would keep applying them and prevent free fall after detachment.
  joint_entity_ = ecm.CreateEntity();
  ecm.CreateComponent(joint_entity_, cmp::DetachableJoint({ base_link_.Entity(), load_link.Entity(), "fixed" }));

  return true;
}

bool GazeboFixedLoadPlugin::detachLoad(std::string& message, gz::sim::EntityComponentManager& ecm)
{
  if (joint_entity_ == gz::sim::kNullEntity || !ecm.HasEntity(joint_entity_)) {
    message = "No fixed load is attached.";
    return false;
  }

  // Removing only the joint preserves the load's pose and velocity for free fall.
  ecm.RequestRemoveEntity(joint_entity_);
  joint_entity_ = gz::sim::kNullEntity;

  return true;
}

void GazeboFixedLoadPlugin::attachLoadCb(
  const AttachSrv::Request::ConstSharedPtr& req,
  const AttachSrv::Response::SharedPtr& res)
{
  Operation operation;
  operation.attach = *req;
  submit(operation);
  res->success = operation.success;
  res->message = operation.message;
}

void GazeboFixedLoadPlugin::detachLoadCb(
  const DetachSrv::Request::ConstSharedPtr&,
  const DetachSrv::Response::SharedPtr& res)
{
  Operation operation;
  submit(operation);
  res->success = operation.success;
  res->message = operation.message;
}
}  // namespace gazebo
}  // namespace tobas

GZ_ADD_PLUGIN(tobas::gazebo::GazeboFixedLoadPlugin, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
