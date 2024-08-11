#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/rotor_speeds.hpp>

#include "../include/tobas_gazebo_plugins/common/common.hpp"
#include "../include/tobas_gazebo_plugins/conversions/gazebo_ros.hpp"

using namespace std;
using namespace gz;
namespace cmp = sim::components;

namespace gazebo
{
class GazeboRotorSpeedsPublisherPlugin : public BaseNode,
                                         public sim::System,
                                         public sim::ISystemConfigure,
                                         public sim::ISystemPostUpdate
{
public:
  explicit GazeboRotorSpeedsPublisherPlugin();

  void Configure(
    const sim::Entity& model,
    const sdf::ElementConstPtr& sdf,
    sim::EntityComponentManager& ecm,
    sim::EventManager&) override;

  void PostUpdate(const sim::UpdateInfo& info, const sim::EntityComponentManager& ecm) override;

private:
  // SDF parameters
  std::vector<std::string> rotor_joint_names_;

  // Gazebo objects
  std::vector<sim::Entity> rotor_joints_;

  // PubSub
  PublisherPtr<tobas_msgs::msg::RotorSpeeds> rotor_speeds_pub_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
};

GazeboRotorSpeedsPublisherPlugin::GazeboRotorSpeedsPublisherPlugin() : BaseNode("rotor_speeds_publisher_plugin")
{
}

void GazeboRotorSpeedsPublisherPlugin::Configure(
  const sim::Entity& model,
  const sdf::ElementConstPtr& sdf,
  sim::EntityComponentManager& ecm,
  sim::EventManager&)
{
  initialize(sdf);
  getSdfParams(sdf);

  // Get the pointer to the rotor joints
  for (const auto& joint_name : rotor_joint_names_)
  {
    const auto joint = ecm.EntityByComponents(cmp::Joint(), cmp::ParentEntity(model), cmp::Name(joint_name));
    if (joint == sim::kNullEntity)
      TOBAS_EXIT("Failed to find specified joint \"", joint_name, "\".");
    rotor_joints_.push_back(joint);
  }

  // Register publishers
  rotor_speeds_pub_ = createPublisher<tobas_msgs::msg::RotorSpeeds>(path::join(ns(), tobas::kRotorSpeedsTopic));
}

void GazeboRotorSpeedsPublisherPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "rotorJointNames", rotor_joint_names_);
}

void GazeboRotorSpeedsPublisherPlugin::PostUpdate(const sim::UpdateInfo& info, const sim::EntityComponentManager& ecm)
{
  // Publish rotor speeds
  auto rotor_speeds = std::make_unique<tobas_msgs::msg::RotorSpeeds>();
  ros2::timeChronoToMsg(info.simTime, rotor_speeds->header.stamp);
  for (const auto& joint : rotor_joints_)
  {
    const auto joint_vel = ecm.Component<cmp::JointVelocity>(joint);
    if (!joint_vel)
      TOBAS_ERROR("Failed to get velocity of joint ", joint);
    const auto rot_speed_sim = joint_vel->Data().at(0);
    const auto rot_speed_real = rot_speed_sim * kRotorSpeedSlowdownSim;
    rotor_speeds->speeds.push_back(abs(rot_speed_real));
  }
  rotor_speeds_pub_->publish(move(rotor_speeds));
}
}  // namespace gazebo

GZ_ADD_PLUGIN(
  gazebo::GazeboRotorSpeedsPublisherPlugin,
  sim::System,
  gazebo::GazeboRotorSpeedsPublisherPlugin::ISystemConfigure,
  gazebo::GazeboRotorSpeedsPublisherPlugin::ISystemPostUpdate)
