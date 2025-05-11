#pragma once

#include "./base.hpp"
#include "tobas_setup_assistant/param_getters/double_pair.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
class EngineDynamicsWidget_Manual : public EngineDynamicsWidget_Base
{
  Q_OBJECT

public:
  explicit EngineDynamicsWidget_Manual();

  const char* name() const override;
  const char* description() const override;

  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  std::pair<double, double> engineConstant() const override;

private:
  ParamGetterWidget_DoublePair* engine_const_;
};
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
