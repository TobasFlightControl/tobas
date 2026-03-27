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

namespace tobas
{
namespace gazebo
{
class GazeboJointEffortControllerPlugin : public BaseNode,
                                          public gz::sim::System,
                                          public gz::sim::ISystemConfigure,
                                          public gz::sim::ISystemPreUpdate
{
  using self = GazeboJointEffortControllerPlugin;

public:
  explicit GazeboJointEffortControllerPlugin();

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
  const cmp::JointTransmittedWrench* jnt_eff_;

  double tar_eff_ = 0.;

  ros2::SubscriberPtr<tobas_gazebo_msgs::msg::JointCommand> cmd_sub_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
  void registerRosInterfaces();

  void commandCb(const tobas_gazebo_msgs::msg::JointCommand::ConstSharedPtr& cmd);
};

GazeboJointEffortControllerPlugin::GazeboJointEffortControllerPlugin()
{
}

void GazeboJointEffortControllerPlugin::Configure(
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

  // Get joint effort
  TOBAS_CHECK(jnt_eff_ = getComponent<cmp::JointTransmittedWrench>(joint_entity, ecm));

  // Reset joint position
  joint_->ResetPosition(ecm, { param_.home_pos });

  // Register ROS interfaces
  registerRosInterfaces();
}

void GazeboJointEffortControllerPlugin::PreUpdate(const gz::sim::UpdateInfo&, gz::sim::EntityComponentManager& ecm)
{
  joint_->SetForce(ecm, { tar_eff_ });
}

void GazeboJointEffortControllerPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "homePosition", param_.home_pos);
}

void GazeboJointEffortControllerPlugin::registerRosInterfaces()
{
  cmd_sub_ = createSubscriber(path::join(kJointCommandTopicNS, joint_name_), &self::commandCb, this);
}

void GazeboJointEffortControllerPlugin::commandCb(const tobas_gazebo_msgs::msg::JointCommand::ConstSharedPtr& cmd)
{
  tar_eff_ = cmd->data;
}
}  // namespace gazebo
}  // namespace tobas

GZ_ADD_PLUGIN(
  tobas::gazebo::GazeboJointEffortControllerPlugin,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)
