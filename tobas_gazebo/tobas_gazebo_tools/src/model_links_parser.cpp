#include "../include/tobas_gazebo_tools/model_links_parser.hpp"
#include "../include/tobas_gazebo_tools/utils.hpp"

using namespace std;
namespace cmp = gz::sim::components;

namespace gazebo
{
ModelLinksParser::ModelLinksParser()
{
}

bool ModelLinksParser::initialize(const gz::sim::Entity& model, const gz::sim::EntityComponentManager& ecm)
{
  if (!ecm.Component<cmp::Model>(model)) {
    ignerr << "Model does not exist." << endl;
    return false;
  }

  links_.clear();

  ecm.Each<cmp::Link, cmp::Name>(
    [&](const gz::sim::Entity& entity, const cmp::Link*, const cmp::Name* name) -> bool
    {
      if (belongsTo(entity, model, ecm)) {
        links_[name->Data()] = entity;
      }
      return true;
    });

  return true;
}
}  // namespace gazebo
