#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>

#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_tools/utils.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_std_tools/check.hpp>

#include <tobas_gazebo_msgs/msg/joint_command.hpp>

#include "tobas_gazebo_system_plugins/common/common.hpp"
#include "tobas_gazebo_system_plugins/node_name.hpp"

namespace ch = std::chrono;
namespace cmp = gz::sim::components;

namespace gazebo
{
class GazeboJointVelocityControllerPlugin : public BaseNode,
                                            public gz::sim::System,
                                            public gz::sim::ISystemConfigure,
                                            public gz::sim::ISystemPreUpdate
{
  using self = GazeboJointVelocityControllerPlugin;

public:
  explicit GazeboJointVelocityControllerPlugin();

  void Configure(
    const gz::sim::Entity& model_entity,
    const sdf::ElementConstPtr& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager&) override;

  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override;

private:
  std::string joint_name_;
  struct Param
  {
    double home_pos;  // [rad]
  } param_;

  std::shared_ptr<gz::sim::Joint> joint_;
  const cmp::JointVelocity* jnt_vel_;

  double tar_vel_ = 0.;

  ros2::SubscriberPtr<tobas_gazebo_msgs::msg::JointCommand> cmd_sub_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
  void registerROSInterfaces();

  void commandCb(const tobas_gazebo_msgs::msg::JointCommand::ConstSharedPtr& cmd);
};

GazeboJointVelocityControllerPlugin::GazeboJointVelocityControllerPlugin()
{
}

void GazeboJointVelocityControllerPlugin::Configure(
  const gz::sim::Entity& model_entity,
  const sdf::ElementConstPtr& sdf,
  gz::sim::EntityComponentManager& ecm,
  gz::sim::EventManager&)
{
  joint_name_ = sdf->Get<std::string>("jointName");
  initialize("gazebo_" + sanitizeNodeName(joint_name_) + "_controller_plugin", sdf);
  getSdfParams(sdf);

  // Get robot model
  const gz::sim::Model model(model_entity);
  if (!model.Valid(ecm)) {
    TOBAS_EXIT("Failed to find model.");
  }

  // Get joint
  const auto joint_entity = model.JointByName(ecm, joint_name_);
  joint_ = std::make_shared<gz::sim::Joint>(joint_entity);
  if (!joint_->Valid(ecm)) {
    TOBAS_EXIT("Failed to find joint \"", joint_name_, "\".");
  }

  // Get joint velocity
  TOBAS_CHECK(jnt_vel_ = getComponent<cmp::JointVelocity>(joint_entity, ecm));

  // Reset joint position
  joint_->ResetPosition(ecm, { param_.home_pos });

  // Register ROS interfaces
  registerROSInterfaces();
}

void GazeboJointVelocityControllerPlugin::PreUpdate(const gz::sim::UpdateInfo&, gz::sim::EntityComponentManager& ecm)
{
  joint_->SetVelocity(ecm, { tar_vel_ });  // これでジョイントにトルクが発生する
}

void GazeboJointVelocityControllerPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "homePosition", param_.home_pos);
}

void GazeboJointVelocityControllerPlugin::registerROSInterfaces()
{
  cmd_sub_ = createSubscriber(path::join(kJointCommandTopicNS, joint_name_), &self::commandCb, this);
}

void GazeboJointVelocityControllerPlugin::commandCb(const tobas_gazebo_msgs::msg::JointCommand::ConstSharedPtr& cmd)
{
  tar_vel_ = cmd->data;
}
}  // namespace gazebo

GZ_ADD_PLUGIN(
  gazebo::GazeboJointVelocityControllerPlugin,
  gz::sim::System,
  gazebo::GazeboJointVelocityControllerPlugin::ISystemConfigure,
  gazebo::GazeboJointVelocityControllerPlugin::ISystemPreUpdate)
