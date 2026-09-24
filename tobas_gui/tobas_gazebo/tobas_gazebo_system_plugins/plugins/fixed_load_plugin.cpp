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
#include <gz/sim/components/Pose.hh>

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
/**
 * @brief Attach a uniform box to an aircraft link using a detachable fixed joint.
 *
 * ROS service callbacks submit a request and wait without a timeout for PreUpdate to process it.
 * Only the simulation thread modifies Gazebo entities and components.
 * A successful response reports an ECM change or removal request;
 * the physics engine applies the corresponding joint change during simulation updates.
 */
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
    std::optional<AttachSrv::Request> attach;  // `nullopt` indicates a detach request.

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
  gz::sim::Entity world_entity_ = gz::sim::kNullEntity;
  gz::sim::Entity link_entity_ = gz::sim::kNullEntity;
  gz::sim::Entity joint_entity_ = gz::sim::kNullEntity;

  std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<Operation> pending_;  // Retain the request and result until `submit()` consumes them.
  bool done_ = false;                 // Protected by `mutex_`; prevents reprocessing a completed request.
  int load_index_ = -1;

  ros2::ServiceServerPtr<AttachSrv> attach_load_ss_;
  ros2::ServiceServerPtr<DetachSrv> detach_load_ss_;

  std::string loadName() const;

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

  world_entity_ = gz::sim::worldEntity(ecm);
  if (world_entity_ == gz::sim::kNullEntity) {
    TOBAS_EXIT("Failed to find the world entity.");
  }

  const gz::sim::Model model(model_entity);
  const auto link_name = getSdfParam<std::string>(sdf, "linkName");
  link_entity_ = model.LinkByName(ecm, link_name);
  if (link_entity_ == gz::sim::kNullEntity) {
    TOBAS_EXIT("Failed to find the specified link '", link_name, "'.");
  }

  creator_.emplace(ecm, events);

  attach_load_ss_ = createService<AttachSrv>(kAttachFixedLoadSrv, &self::attachLoadCb, this);
  detach_load_ss_ = createService<DetachSrv>(kDetachFixedLoadSrv, &self::detachLoadCb, this);
}

void GazeboFixedLoadPlugin::PreUpdate(const gz::sim::UpdateInfo&, gz::sim::EntityComponentManager& ecm)
{
  std::lock_guard lock(mutex_);

  if (!pending_ || done_) {
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

std::string GazeboFixedLoadPlugin::loadName() const
{
  return "fixed_load_" + std::to_string(link_entity_) + "_" + std::to_string(load_index_);
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

  // Release the mutex while waiting so `PreUpdate` can execute the request.
  cv_.wait(lock, [this]() { return done_; });

  // Consume the result before allowing another request to replace it.
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

  // Compose the attachment link's world pose with the load pose relative to that link.
  // W: world frame, B: attachment link frame, L: load frame centered at its center of mass.
  gz::math::Pose3d T_B_L;
  gazebo::poseRosToGazebo(req.load_pose, T_B_L);
  const auto T_W_B = gz::sim::worldPose(link_entity_, ecm);
  const auto T_W_L = T_W_B * T_B_L;

  // Increment the index to avoid duplicate model names.
  ++load_index_;
  const auto load_name = loadName();

  // Generate and parse the SDF for a uniform box.
  sdf::Root root;
  const auto sdf = makeBoxSdf(load_name, req.load_size.x, req.load_size.y, req.load_size.z, req.load_mass);
  const auto errors = root.LoadSdfString(sdf);
  if (!errors.empty() || !root.Model()) {
    message = "Failed to construct the load SDF.";
    return false;
  }

  // Create the load model and its child entities.
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
  ecm.CreateComponent(joint_entity_, cmp::DetachableJoint({ link_entity_, load_link.Entity(), "fixed" }));

  return true;
}

bool GazeboFixedLoadPlugin::detachLoad(std::string& message, gz::sim::EntityComponentManager& ecm)
{
  if (joint_entity_ == gz::sim::kNullEntity || !ecm.HasEntity(joint_entity_)) {
    message = "No fixed load is attached.";
    return false;
  }

  // Request removal of only the joint; leave the load model and its state intact.
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

  if (operation.success) {
    TOBAS_INFO(loadName(), " has been created and attached to the vehicle successfully.");
  }
}

void GazeboFixedLoadPlugin::detachLoadCb(
  const DetachSrv::Request::ConstSharedPtr&,
  const DetachSrv::Response::SharedPtr& res)
{
  Operation operation;
  submit(operation);
  res->success = operation.success;
  res->message = operation.message;

  if (operation.success) {
    TOBAS_INFO(loadName(), " has been detached from the vehicle successfully.");
  }
}
}  // namespace gazebo
}  // namespace tobas

GZ_ADD_PLUGIN(tobas::gazebo::GazeboFixedLoadPlugin, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
