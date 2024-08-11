#include <tobas_gazebo_msgs/ContactStates.h>

#include "./world_contacts_plugin.hpp"
#include "../include/tobas_gazebo_plugins/common/common.hpp"

#include "../include/tobas_gazebo_plugins/conversions/gazebo_ros.hpp"

using namespace std;
using namespace gz;
namespace cmp = sim::components;

namespace gazebo
{
GazeboWorldContactsPlugin::GazeboWorldContactsPlugin() : super()
{
}

void GazeboWorldContactsPlugin::Configure(
  const sim::Entity& model,
  const sdf::ElementConstPtr& sdf,
  sim::EntityComponentManager& ecm,
  sim::EventManager&)
{initialize(sdf);


  getSdfParams(sdf);

  contact_manager_ = model->GetWorld()->Physics()->GetContactManager();
  contact_manager_->SetNeverDropContacts(true);  // 購読者がいなくても接触情報を保持するようにする

  contacts_pub_ = createPublisher<tobas_gazebo_msgs::ContactStates>(path::join(ns(), kContactStatesTopic);

  update_connection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&self::onUpdate, this, _1));
}

void GazeboWorldContactsPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{

}

void GazeboWorldContactsPlugin::PostUpdate(const sim::UpdateInfo& info, const sim::EntityComponentManager& ecm)
{
  const auto msg =std::make_unique<tobas_gazebo_msgs::ContactStates>();

  ros2::timeChronoToMsg(info.simTime, msg->header.stamp);

  const auto num_contacts = contact_manager_->GetContactCount();
  msg->states.resize(num_contacts);

  for (size_t i = 0; i < num_contacts; ++i)
  {
    const auto contact = contact_manager_->GetContact(i);

    const auto& col1 = contact->collision1;
    msg->states[i].collision1.name = col1->GetModel()->GetName();
    msg->states[i].collision1.shape_type = col1->GetShapeType();

    const auto& col2 = contact->collision2;
    msg->states[i].collision2.name = col2->GetModel()->GetName();
    msg->states[i].collision2.shape_type = col2->GetShapeType();
  }

  contacts_pub_->publish(msg);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboWorldContactsPlugin);
}  // namespace gazebo
