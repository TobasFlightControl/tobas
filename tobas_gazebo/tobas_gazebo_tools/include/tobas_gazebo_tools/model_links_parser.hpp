#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/components.hh>

namespace gazebo
{
  /* モデルに属する全てのリンクを保持する． */
class ModelLinksParser
{
public:
  explicit ModelLinksParser();

  bool initialize(const gz::sim::Entity& model, const gz::sim::EntityComponentManager& ecm);

  inline const std::map<std::string, gz::sim::Entity>& getLinks() const
  {
    return links_;
  }

  inline const gz::sim::Entity& getLink(const std::string& link_name) const
  {
    return links_.at(link_name);
  }

private:
  std::map<std::string, gz::sim::Entity> links_;
};
}  // namespace gazebo
