#include <gz/transport/Node.hh>
#include <gz/msgs/vector3d.pb.h>

#include <tobas_std_tools/geometry.hpp>
#include <tobas_ros2_tools/time.hpp>

#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_tools/conversion.hpp>
#include <tobas_gazebo_tools/utils.hpp>

#include "../include/tobas_gazebo_plugins/common/common.hpp"

using namespace std;
namespace cmp = gz::sim::components;

namespace gazebo
{
class GazeboLookAtPositionPlugin : public BaseNode,
                                   public gz::sim::System,
                                   public gz::sim::ISystemConfigure,
                                   public gz::sim::ISystemPostUpdate
{
  using self = GazeboLookAtPositionPlugin;

public:
  explicit GazeboLookAtPositionPlugin();

  void Configure(
    const gz::sim::Entity& model,
    const sdf::ElementConstPtr& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager&) override;

  void PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) override;

private:
  // SDF parameters
  std::string link_name_;

  const cmp::WorldPose* pose_W_;

  gz::transport::Node node_;
  gz::msgs::Vector3d lookat_pos_;
  gz::transport::Node::Publisher lookat_pos_pub_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
};

GazeboLookAtPositionPlugin::GazeboLookAtPositionPlugin()
{
}

void GazeboLookAtPositionPlugin::Configure(
  const gz::sim::Entity& model,
  const sdf::ElementConstPtr& sdf,
  gz::sim::EntityComponentManager& ecm,
  gz::sim::EventManager&)
{
  initialize("gazebo_lookat_position_plugin", sdf);
  getSdfParams(sdf);

  const auto link = ecm.EntityByComponents(cmp::Link(), cmp::ParentEntity(model), cmp::Name(link_name_));
  if (link == gz::sim::kNullEntity)
    TOBAS_EXIT("Failed to find specified link \"", link_name_, "\".");

  pose_W_ = getComponent<cmp::WorldPose>(link, ecm);

  lookat_pos_pub_ = node_.Advertise<gz::msgs::Vector3d>(kLookAtPositionTopic);
}

void GazeboLookAtPositionPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "linkName", link_name_);
}

void GazeboLookAtPositionPlugin::PostUpdate(const gz::sim::UpdateInfo&, const gz::sim::EntityComponentManager&)
{
  vector3dGzToMsg(pose_W_->Data().Pos(), lookat_pos_);
  lookat_pos_pub_.Publish(lookat_pos_);
}
}  // namespace gazebo

GZ_ADD_PLUGIN(
  gazebo::GazeboLookAtPositionPlugin,
  gz::sim::System,
  gazebo::GazeboLookAtPositionPlugin::ISystemConfigure,
  gazebo::GazeboLookAtPositionPlugin::ISystemPostUpdate)
