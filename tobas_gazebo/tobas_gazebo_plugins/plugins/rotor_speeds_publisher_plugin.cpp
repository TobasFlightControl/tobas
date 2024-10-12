#include <tobas_ros2_tools/time.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/rotor_speeds.hpp>

#include "../include/tobas_gazebo_plugins/common/common.hpp"
#include "../include/tobas_gazebo_plugins/conversions/gazebo_msg.hpp"
#include "../include/tobas_gazebo_plugins/utils.hpp"

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
  vector<string> rotor_joint_names_;

  // Gazebo objects
  vector<cmp::JointVelocity*> rotor_jntvels_;

  // PubSub
  ros2::PublisherPtr<tobas_msgs::msg::RotorSpeeds> rotor_speeds_pub_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
};

GazeboRotorSpeedsPublisherPlugin::GazeboRotorSpeedsPublisherPlugin()
{
}

void GazeboRotorSpeedsPublisherPlugin::Configure(
  const sim::Entity& model,
  const sdf::ElementConstPtr& sdf,
  sim::EntityComponentManager& ecm,
  sim::EventManager&)
{
  initialize("gazebo_rotor_speeds_publisher_plugin", sdf);
  getSdfParams(sdf);

  // Get the pointer to the rotor joints
  for (const auto& joint_name : rotor_joint_names_)
  {
    const auto joint = ecm.EntityByComponents(cmp::Joint(), cmp::ParentEntity(model), cmp::Name(joint_name));
    if (joint == sim::kNullEntity)
      TOBAS_EXIT("Failed to find specified joint \"", joint_name, "\".");
    const auto jntvel = getComponent<cmp::JointVelocity>(joint, ecm);
    rotor_jntvels_.push_back(jntvel);
  }

  // Register publishers
  rotor_speeds_pub_ = createPublisher<tobas_msgs::msg::RotorSpeeds>(tobas::kRotorSpeedsTopic);
}

void GazeboRotorSpeedsPublisherPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "rotorJointNames", rotor_joint_names_);
}

void GazeboRotorSpeedsPublisherPlugin::PostUpdate(const sim::UpdateInfo& info, const sim::EntityComponentManager&)
{
  // Publish rotor speeds
  auto rotor_speeds = make_unique<tobas_msgs::msg::RotorSpeeds>();
  ros2::timeChronoToMsg(info.simTime, rotor_speeds->header.stamp);
  for (const auto& jntvel : rotor_jntvels_)
  {
    const auto rot_speed_sim = jntvel->Data().at(0);
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
