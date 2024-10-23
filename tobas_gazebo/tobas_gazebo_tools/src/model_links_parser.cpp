#include "../include/tobas_gazebo_tools/model_links_parser.hpp"
#include "../include/tobas_gazebo_tools/utils.hpp"

using namespace std;
using namespace gz::sim;

namespace gazebo
{
ModelLinksParser::ModelLinksParser()
{
}

bool ModelLinksParser::initialize(const Entity& model, const EntityComponentManager& ecm)
{
  if (!ecm.Component<components::Model>(model))
  {
    ignerr << "Model does not exist." << endl;
    return false;
  }

  links_.clear();

  ecm.Each<components::Link, components::Name>(
    [&](const Entity& entity, const components::Link*, const components::Name* name) -> bool
    {
      if (belongsTo(entity, model, ecm))
        links_[name->Data()] = entity;
      return true;
    });

  return true;
}
}  // namespace gazebo
