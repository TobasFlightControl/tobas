#pragma once

#include "./base.hpp"

namespace gui
{
namespace sa
{
namespace mission
{
class NoneWidget : public BaseExecutorWidget
{
  Q_OBJECT

public:
  explicit NoneWidget();

  QString executorPackage() const override;
  QString pluginName() const override;

  YAML::Node staticParams() const override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  bool isValid() override;
};
}  // namespace mission
}  // namespace sa
}  // namespace gui
