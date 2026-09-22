// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

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
#include <sdf/Model.hh>
#include <sdf/Root.hh>

#include <tobas_gazebo_common/constants.hpp>

#include <tobas_gazebo_msgs/srv/attach_fixed_load.hpp>
#include <tobas_gazebo_msgs/srv/detach_fixed_load.hpp>

#include "tobas_gazebo_system_plugins/common/common.hpp"
#include "tobas_gazebo_system_plugins/inertia.hpp"
#include "tobas_gazebo_system_plugins/sdf_string.hpp"

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
  using AttachSrv = tobas_gazebo_msgs::srv::AttachFixedLoad;
  using DetachSrv = tobas_gazebo_msgs::srv::DetachFixedLoad;

  struct Operation
  {
    std::optional<AttachSrv::Request> attach;
    bool success = false;
    std::string message;
    bool done = false;
  };

  struct Requests
  {
    std::mutex mutex;
    std::condition_variable cv;
    std::shared_ptr<Operation> pending;
    bool stopping = false;
  };

public:
  ~GazeboFixedLoadPlugin() override;

  void Configure(
    const gz::sim::Entity& model,
    const sdf::ElementConstPtr& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager& events) override;

  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override;

private:
  static void submit(const std::shared_ptr<Requests>& requests, Operation& operation);
  void attachLoad(const AttachSrv::Request& req, Operation& result, gz::sim::EntityComponentManager& ecm);

  std::shared_ptr<Requests> requests_ = std::make_shared<Requests>();
  std::unique_ptr<gz::sim::SdfEntityCreator> creator_;
  gz::sim::Entity world_ = gz::sim::kNullEntity;
  gz::sim::Link base_link_;
  gz::sim::Entity joint_ = gz::sim::kNullEntity;
  int load_index_ = 0;

  ros2::ServiceServerPtr<AttachSrv> attach_load_ss_;
  ros2::ServiceServerPtr<DetachSrv> detach_load_ss_;
};

GazeboFixedLoadPlugin::~GazeboFixedLoadPlugin()
{
  // Callbacks capture shared state, so they remain safe until BaseNode stops its executor.
  const std::lock_guard lock(requests_->mutex);
  requests_->stopping = true;
  requests_->cv.notify_all();
}

void GazeboFixedLoadPlugin::Configure(
  const gz::sim::Entity& model,
  const sdf::ElementConstPtr& sdf,
  gz::sim::EntityComponentManager& ecm,
  gz::sim::EventManager& events)
{
  initialize("gazebo_fixed_load_plugin", sdf);

  std::string link_name;
  getSdfParam(sdf, "linkName", link_name);
  base_link_ = gz::sim::Link(gz::sim::Model(model).LinkByName(ecm, link_name));
  world_ = ecm.EntityByComponents(cmp::World());
  if (!base_link_.Valid(ecm) || world_ == gz::sim::kNullEntity) {
    TOBAS_EXIT("Failed to find the world or attachment link '", link_name, "'.");
  }
  creator_ = std::make_unique<gz::sim::SdfEntityCreator>(ecm, events);

  // The ROS executor must never access Gazebo entities or components directly.
  attach_load_ss_ = node_->create_service<AttachSrv>(
    kAttachFixedLoadSrv,
    [requests = requests_](const AttachSrv::Request::SharedPtr req, const AttachSrv::Response::SharedPtr res)
    {
      Operation operation;
      operation.attach = *req;
      submit(requests, operation);
      res->success = operation.success;
      res->message = operation.message;
    });
  detach_load_ss_ = node_->create_service<DetachSrv>(
    kDetachFixedLoadSrv,
    [requests = requests_](const DetachSrv::Request::SharedPtr, const DetachSrv::Response::SharedPtr res)
    {
      Operation operation;
      submit(requests, operation);
      res->success = operation.success;
      res->message = operation.message;
    });
}

void GazeboFixedLoadPlugin::submit(const std::shared_ptr<Requests>& requests, Operation& operation)
{
  std::unique_lock lock(requests->mutex);
  if (requests->stopping || requests->pending) {
    operation.message = "Plugin is stopping or another request is pending.";
    return;
  }

  const auto pending = std::make_shared<Operation>(operation);
  requests->pending = pending;
  const auto ready =
    requests->cv.wait_for(lock, std::chrono::seconds(2), [&]() { return pending->done || requests->stopping; });
  if (!ready || !pending->done) {
    // Cancel before replying: a timed-out request must not attach a load later.
    requests->pending.reset();
    operation.message = "No simulation update completed the request. Run the simulation and retry.";
    return;
  }
  operation = *pending;
}

void GazeboFixedLoadPlugin::PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm)
{
  if (info.paused) {
    return;
  }

  const std::lock_guard lock(requests_->mutex);
  if (!requests_->pending || requests_->stopping) {
    return;
  }
  auto& operation = *requests_->pending;
  if (!base_link_.Valid(ecm)) {
    operation.message = "Attachment link no longer exists.";
  }
  else if (operation.attach) {
    attachLoad(*operation.attach, operation, ecm);
  }
  else if (joint_ == gz::sim::kNullEntity || !ecm.HasEntity(joint_)) {
    joint_ = gz::sim::kNullEntity;
    operation.message = "No fixed load is attached.";
  }
  else {
    // Removing only the joint preserves the load's pose and velocity for free fall.
    ecm.RequestRemoveEntity(joint_);
    joint_ = gz::sim::kNullEntity;
    operation.success = true;
  }
  operation.done = true;
  requests_->pending.reset();
  requests_->cv.notify_all();
}

void GazeboFixedLoadPlugin::attachLoad(
  const AttachSrv::Request& req,
  Operation& result,
  gz::sim::EntityComponentManager& ecm)
{
  if (joint_ != gz::sim::kNullEntity && ecm.HasEntity(joint_)) {
    result.message = "A fixed load is already attached.";
    return;
  }
  const auto& size = req.load_size;
  for (const auto value : { size.x, size.y, size.z, req.load_mass }) {
    if (!std::isfinite(value) || value <= 0.0) {
      result.message = "Load dimensions and mass must be finite and positive.";
      return;
    }
  }
  const auto& center = req.load_pose.position;
  if (!std::isfinite(center.x) || !std::isfinite(center.y) || !std::isfinite(center.z)) {
    result.message = "Load center of mass must be finite.";
    return;
  }
  const auto& orientation = req.load_pose.orientation;
  if (
    !std::isfinite(orientation.w) || !std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
    !std::isfinite(orientation.z)) {
    result.message = "Load orientation must be finite.";
    return;
  }
  const auto norm = std::hypot(std::hypot(orientation.w, orientation.x), std::hypot(orientation.y, orientation.z));
  if (!std::isfinite(norm) || norm <= 0.0) {
    result.message = "Load orientation quaternion must have a finite, nonzero norm.";
    return;
  }
  const auto [ixx, iyy, izz] = boxInertia(size.x, size.y, size.z, req.load_mass);
  for (const auto value : { ixx, iyy, izz }) {
    if (!std::isfinite(value) || value <= 0.0) {
      result.message = "Load dimensions and mass produce invalid inertia.";
      return;
    }
  }

  const gz::math::Pose3d relative_pose(
    gz::math::Vector3d(center.x, center.y, center.z),
    gz::math::Quaterniond(orientation.w / norm, orientation.x / norm, orientation.y / norm, orientation.z / norm));
  const auto load_pose = gz::sim::worldPose(base_link_.Entity(), ecm) * relative_pose;

  // Include the attachment entity to avoid collisions between multiple aircraft.
  std::string load_name;
  do {
    load_name = "fixed_load_" + std::to_string(base_link_.Entity()) + "_" + std::to_string(++load_index_);
  } while (ecm.EntityByComponents(cmp::Model(), cmp::Name(load_name)) != gz::sim::kNullEntity);

  sdf::Root root;
  const auto errors = root.LoadSdfString(makeBoxSdf(load_name, size.x, size.y, size.z, req.load_mass, 0.0, 0.0, 0.0));
  if (!errors.empty() || !root.Model()) {
    result.message = "Failed to construct the load SDF.";
    return;
  }
  const auto load_entity = creator_->CreateEntities(root.Model());
  if (load_entity == gz::sim::kNullEntity) {
    result.message = "Failed to create the load model.";
    return;
  }
  creator_->SetParent(load_entity, world_);
  // Set the ECM pose before physics sees the new model. Changing the parsed SDF's
  // raw pose alone does not update its already-resolved semantic pose graph.
  ecm.SetComponentData<cmp::Pose>(load_entity, load_pose);
  const gz::sim::Link load_link(gz::sim::Model(load_entity).CanonicalLink(ecm));
  if (!load_link.Valid(ecm)) {
    creator_->RequestRemoveEntity(load_entity);
    result.message = "Failed to create the load link.";
    return;
  }

  // The fixed joint makes the load follow the parent's rigid-body velocity,
  // including rotation about the offset. Do not leave velocity command components:
  // Gazebo would keep applying them and prevent free fall after detachment.
  joint_ = ecm.CreateEntity();
  ecm.CreateComponent(joint_, cmp::DetachableJoint({ base_link_.Entity(), load_link.Entity(), "fixed" }));
  result.success = true;
}
}  // namespace gazebo
}  // namespace tobas

GZ_ADD_PLUGIN(tobas::gazebo::GazeboFixedLoadPlugin, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
