#pragma once

#include "./model_links_parser.hpp"

namespace gazebo
{
/* モデルの合計質量を保持する． */
class ModelMassHolder
{
public:
  explicit ModelMassHolder();

  bool initialize(const gz::sim::Entity& model, const gz::sim::EntityComponentManager& ecm);

  inline const double& getMass() const
  {
    return mass_;
  }

private:
  double mass_;
  ModelLinksParser model_links_parser_;
};
}  // namespace gazebo
