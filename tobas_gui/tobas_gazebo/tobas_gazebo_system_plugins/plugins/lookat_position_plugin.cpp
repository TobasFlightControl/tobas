// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <gz/msgs/vector3d.pb.h>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>

#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_conversions/gazebo_msg.hpp>
#include <tobas_gazebo_tools/utils.hpp>

#include "tobas_gazebo_system_plugins/common/common.hpp"

namespace cmp = gz::sim::components;

namespace tobas
{
namespace gazebo
{
class GazeboLookAtPositionPlugin : public BaseNode,
                                   public gz::sim::System,
                                   public gz::sim::ISystemConfigure,
                                   public gz::sim::ISystemPostUpdate
{
public:
  explicit GazeboLookAtPositionPlugin();

  void Configure(
    const gz::sim::Entity& model_entity,
    const sdf::ElementConstPtr& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager&) override;

  void PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) override;

private:
  const cmp::WorldPose* pose_W_;

  gz::transport::Node node_;
  gz::msgs::Vector3d lookat_pos_;
  gz::transport::Node::Publisher lookat_pos_pub_;
};

GazeboLookAtPositionPlugin::GazeboLookAtPositionPlugin()
{
}

void GazeboLookAtPositionPlugin::Configure(
  const gz::sim::Entity& model_entity,
  const sdf::ElementConstPtr& sdf,
  gz::sim::EntityComponentManager& ecm,
  gz::sim::EventManager&)
{
  initialize("gazebo_lookat_position_plugin", sdf);

  const gz::sim::Model model(model_entity);
  const auto link_name = getSdfParam<std::string>(sdf, "linkName");
  const auto link_entity = model.LinkByName(ecm, link_name);
  if (link_entity == gz::sim::kNullEntity) {
    TOBAS_EXIT("Failed to find specified link '", link_name, "'.");
  }

  pose_W_ = getComponent<cmp::WorldPose>(link_entity, ecm);

  lookat_pos_pub_ = node_.Advertise<gz::msgs::Vector3d>(kGzCameraLookAtTopic);
}

void GazeboLookAtPositionPlugin::PostUpdate(const gz::sim::UpdateInfo&, const gz::sim::EntityComponentManager&)
{
  vector3dGzToMsg(pose_W_->Data().Pos(), lookat_pos_);
  lookat_pos_pub_.Publish(lookat_pos_);
}
}  // namespace gazebo
}  // namespace tobas

GZ_ADD_PLUGIN(
  tobas::gazebo::GazeboLookAtPositionPlugin,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPostUpdate)
