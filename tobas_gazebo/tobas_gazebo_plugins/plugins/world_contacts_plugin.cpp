#include <tobas_gazebo_msgs/ContactStates.h>

#include "./world_contacts_plugin.hpp"
#include "../include/tobas_gazebo_plugins/common.hpp"
#include "../include/tobas_gazebo_plugins/sdfparam.hpp"
#include "../include/tobas_gazebo_plugins/conversions/gazebo_ros.hpp"

using namespace std;
using namespace gz::math;

namespace gazebo
{
GazeboWorldContactsPlugin::GazeboWorldContactsPlugin() : super()
{
}

void GazeboWorldContactsPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  gzmsg << "Loading " << kPluginName << "." << endl;

  getSdfParams(sdf);

  contact_manager_ = model->GetWorld()->Physics()->GetContactManager();
  contact_manager_->SetNeverDropContacts(true);  // 購読者がいなくても接触情報を保持するようにする

  contacts_pub_ = createPublisher<tobas_gazebo_msgs::ContactStates>("/" + ns() + "/" + kContactStatesTopic);

  update_connection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&self::onUpdate, this, _1));
}

void GazeboWorldContactsPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "robotNamespace", ns());
}

void GazeboWorldContactsPlugin::onUpdate(const common::UpdateInfo& info)
{
  const auto msg =std::make_unique<tobas_gazebo_msgs::ContactStates>();

  timeGazeboToRos(info.simTime, msg->header.stamp);

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
