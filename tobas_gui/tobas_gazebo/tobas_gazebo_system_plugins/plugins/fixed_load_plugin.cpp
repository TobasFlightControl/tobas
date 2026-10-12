// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/CollisionBitmask.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Pose.hh>

#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_conversions/gazebo_ros.hpp>

#include <tobas_gazebo_msgs/srv/attach_fixed_load.hpp>
#include <tobas_gazebo_msgs/srv/detach_fixed_load.hpp>

#include "tobas_gazebo_system_plugins/common/common.hpp"
#include "tobas_gazebo_system_plugins/sdf_string.hpp"

namespace cmp = gz::sim::components;

namespace tobas
{
namespace gazebo
{
namespace
{
// Keep aircraft and fixed loads in disjoint collision groups.
// Default world collisions (0xFFFF) intersect both groups, so ground contact is preserved.
// These groups apply to every instance of this plugin, including released loads.
constexpr uint16_t kAircraftCollisionMask = 0x0001;
constexpr uint16_t kLoadCollisionMask = 0x0002;

void setModelCollisionMask(gz::sim::Entity _model, uint16_t _mask, gz::sim::EntityComponentManager& _ecm)
{
  // Include every link and nested model, without changing visual geometry.
  for (const auto entity : _ecm.Descendants(_model)) {
    if (!_ecm.Component<cmp::Collision>(entity)) {
      continue;
    }

    // Gazebo applies these commands before the next physics step. Set both:
    // collision is enabled when (category1 & collide2) | (category2 & collide1) is nonzero,
    // so an existing category mask must not bypass the exclusion.
    _ecm.SetComponentData<cmp::CollideBitmaskCmd>(entity, _mask);
    _ecm.SetComponentData<cmp::CategoryBitmaskCmd>(entity, _mask);
  }
}
}  // namespace

/**
 * Attach a uniform box to an aircraft link using a detachable fixed joint.
 *
 * Aircraft collisions use mask 0x0001 and fixed loads use 0x0002 for both category and collide masks.
 * The groups never collide, even after detachment.
 * Environment masks must intersect the corresponding group for contact.
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
    std::optional<AttachSrv::Request> attach;  ///< `nullopt` indicates a detach request.

    // Response
    bool success = false;
    std::string message;
  };

public:
  void Configure(
    const gz::sim::Entity& _model,
    const sdf::ElementConstPtr& _sdf,
    gz::sim::EntityComponentManager& _ecm,
    gz::sim::EventManager& _events) override;

  void PreUpdate(const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm) override;

private:
  std::optional<gz::sim::SdfEntityCreator> creator_;
  gz::sim::Entity model_entity_ = gz::sim::kNullEntity;
  gz::sim::Entity world_entity_ = gz::sim::kNullEntity;
  gz::sim::Entity link_entity_ = gz::sim::kNullEntity;
  gz::sim::Entity joint_entity_ = gz::sim::kNullEntity;

  std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<Operation> pending_;  ///< Retain the request and result until `submit()` consumes them.
  bool done_ = false;                 ///< Protected by `mutex_`; prevents reprocessing a completed request.
  int load_index_ = -1;

  ros2::ServiceServerPtr<AttachSrv> attach_load_ss_;
  ros2::ServiceServerPtr<DetachSrv> detach_load_ss_;

  std::string loadName() const;

  void submit(Operation& _operation);
  bool attachLoad(const AttachSrv::Request& _req, std::string& _message, gz::sim::EntityComponentManager& _ecm);
  bool detachLoad(std::string& _message, gz::sim::EntityComponentManager& _ecm);

  void attachLoadCb(const AttachSrv::Request::ConstSharedPtr& _req, const AttachSrv::Response::SharedPtr& _res);
  void detachLoadCb(const DetachSrv::Request::ConstSharedPtr& _req, const DetachSrv::Response::SharedPtr& _res);
};

void GazeboFixedLoadPlugin::Configure(
  const gz::sim::Entity& _model_entity,
  const sdf::ElementConstPtr& _sdf,
  gz::sim::EntityComponentManager& _ecm,
  gz::sim::EventManager& _events)
{
  initialize("gazebo_fixed_load_plugin", _sdf);
  model_entity_ = _model_entity;

  world_entity_ = gz::sim::worldEntity(_ecm);
  if (world_entity_ == gz::sim::kNullEntity) {
    TOBAS_EXIT("Failed to find the world entity.");
  }

  const gz::sim::Model model(_model_entity);
  const auto link_name = getSdfParam<std::string>(_sdf, "linkName");
  link_entity_ = model.LinkByName(_ecm, link_name);
  if (link_entity_ == gz::sim::kNullEntity) {
    TOBAS_EXIT("Failed to find the specified link '", link_name, "'.");
  }

  creator_.emplace(_ecm, _events);

  attach_load_ss_ = createService<AttachSrv>(kAttachFixedLoadSrv, &self::attachLoadCb, this);
  detach_load_ss_ = createService<DetachSrv>(kDetachFixedLoadSrv, &self::detachLoadCb, this);
}

void GazeboFixedLoadPlugin::PreUpdate(const gz::sim::UpdateInfo&, gz::sim::EntityComponentManager& _ecm)
{
  std::lock_guard lock(mutex_);

  if (!pending_ || done_) {
    return;
  }

  if (pending_->attach) {
    pending_->success = attachLoad(*pending_->attach, pending_->message, _ecm);
  }
  else {
    pending_->success = detachLoad(pending_->message, _ecm);
  }

  done_ = true;
  cv_.notify_all();
}

std::string GazeboFixedLoadPlugin::loadName() const
{
  return "fixed_load_" + std::to_string(link_entity_) + "_" + std::to_string(load_index_);
}

void GazeboFixedLoadPlugin::submit(Operation& _operation)
{
  std::unique_lock lock(mutex_);

  if (pending_) {
    _operation.success = false;
    _operation.message = "Another request is pending.";
    return;
  }

  pending_ = _operation;
  done_ = false;

  // Release the mutex while waiting so `PreUpdate` can execute the request.
  cv_.wait(lock, [this]() { return done_; });

  // Consume the result before allowing another request to replace it.
  _operation = *pending_;
  pending_.reset();
}

bool GazeboFixedLoadPlugin::attachLoad(
  const AttachSrv::Request& _req,
  std::string& _message,
  gz::sim::EntityComponentManager& _ecm)
{
  if (joint_entity_ != gz::sim::kNullEntity && _ecm.HasEntity(joint_entity_)) {
    _message = "A fixed load is already attached.";
    return false;
  }

  if (_req.load_size.x <= 0.0 || _req.load_size.y <= 0.0 || _req.load_size.z <= 0.0) {
    _message = "Load dimensions must be positive.";
    return false;
  }
  if (_req.load_mass <= 0.0) {
    _message = "Load mass must be positive.";
    return false;
  }

  // Compose the attachment link's world pose with the load pose relative to that link.
  // W: world frame, B: attachment link frame, L: load frame centered at its center of mass.
  gz::math::Pose3d T_B_L;
  gazebo::poseRosToGazebo(_req.load_pose, T_B_L);
  const auto T_W_B = gz::sim::worldPose(link_entity_, _ecm);
  const auto T_W_L = T_W_B * T_B_L;

  // Increment the index to avoid duplicate model names.
  ++load_index_;
  const auto load_name = loadName();

  // Generate and parse the SDF for a uniform box.
  sdf::Root root;
  const auto sdf = makeBoxSdf(load_name, _req.load_size.x, _req.load_size.y, _req.load_size.z, _req.load_mass);
  const auto errors = root.LoadSdfString(sdf);
  if (!errors.empty() || !root.Model()) {
    _message = "Failed to construct the load SDF.";
    return false;
  }

  // Create the load model and its child entities.
  const auto load_entity = creator_->CreateEntities(root.Model());
  if (load_entity == gz::sim::kNullEntity) {
    _message = "Failed to create the load model.";
    return false;
  }
  creator_->SetParent(load_entity, world_entity_);

  // Set the ECM pose before physics sees the new model.
  // Changing the parsed SDF's raw pose alone does not update its already-resolved semantic pose graph.
  _ecm.SetComponentData<cmp::Pose>(load_entity, T_W_L);
  const gz::sim::Link load_link(gz::sim::Model(load_entity).CanonicalLink(_ecm));
  if (!load_link.Valid(_ecm)) {
    creator_->RequestRemoveEntity(load_entity);
    _message = "Failed to create the load link.";
    return false;
  }

  // Allow overlapping placement without generating contacts with the aircraft.
  // Apply both groups in this `PreUpdate`, before physics can step the new load.
  setModelCollisionMask(model_entity_, kAircraftCollisionMask, _ecm);
  setModelCollisionMask(load_entity, kLoadCollisionMask, _ecm);

  // The fixed joint makes the load follow the parent's rigid-body velocity,
  // including rotation about the offset. Do not leave velocity command components:
  // Gazebo would keep applying them and prevent free fall after detachment.
  joint_entity_ = _ecm.CreateEntity();
  _ecm.CreateComponent(joint_entity_, cmp::DetachableJoint({ link_entity_, load_link.Entity(), "fixed" }));

  return true;
}

bool GazeboFixedLoadPlugin::detachLoad(std::string& _message, gz::sim::EntityComponentManager& _ecm)
{
  if (joint_entity_ == gz::sim::kNullEntity || !_ecm.HasEntity(joint_entity_)) {
    _message = "No fixed load is attached.";
    return false;
  }

  // Request removal of only the joint; leave the load model and its state intact.
  _ecm.RequestRemoveEntity(joint_entity_);
  joint_entity_ = gz::sim::kNullEntity;

  return true;
}

void GazeboFixedLoadPlugin::attachLoadCb(
  const AttachSrv::Request::ConstSharedPtr& _req,
  const AttachSrv::Response::SharedPtr& _res)
{
  Operation operation;
  operation.attach = *_req;
  submit(operation);
  _res->success = operation.success;
  _res->message = operation.message;

  if (operation.success) {
    TOBAS_INFO(loadName(), " has been created and attached to the vehicle successfully.");
  }
}

void GazeboFixedLoadPlugin::detachLoadCb(
  const DetachSrv::Request::ConstSharedPtr&,
  const DetachSrv::Response::SharedPtr& _res)
{
  Operation operation;
  submit(operation);
  _res->success = operation.success;
  _res->message = operation.message;

  if (operation.success) {
    TOBAS_INFO(loadName(), " has been detached from the vehicle successfully.");
  }
}
}  // namespace gazebo
}  // namespace tobas

GZ_ADD_PLUGIN(tobas::gazebo::GazeboFixedLoadPlugin, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
