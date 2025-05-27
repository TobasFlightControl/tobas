#include "tobas_gazebo_tools/model_mass_holder.hpp"

using namespace std;
using namespace gz::sim;

namespace gazebo
{
ModelMassHolder::ModelMassHolder()
{
}

bool ModelMassHolder::initialize(const gz::sim::Entity& model, const gz::sim::EntityComponentManager& ecm)
{
  if (!model_links_parser_.initialize(model, ecm)) {
    gzerr << "Failed to initialize model links parser." << endl;
    return false;
  }

  for (const auto& [link_name, link_entity] : model_links_parser_.getLinks()) {
    const auto inertial_entity = ecm.Component<components::Inertial>(link_entity);
    if (!inertial_entity) {
      gzerr << "Failed to get the inertial entity of \"" << link_name << "\".";
      return false;
    }

    const auto& inertial = inertial_entity->Data();
    mass_ += inertial.MassMatrix().Mass();
  }

  return true;
}
}  // namespace gazebo
