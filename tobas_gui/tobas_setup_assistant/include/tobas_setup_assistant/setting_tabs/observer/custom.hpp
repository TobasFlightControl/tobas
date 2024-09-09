#pragma once

#include "./base.hpp"

namespace gui
{
namespace setup_assistant
{
class CustomObserverWidget : public BaseObserverWidget
{
  Q_OBJECT

public:
  explicit CustomObserverWidget();

  const char* name() const override;
  const char* description() const override;
  const char* observerPackage() const override;

  YAML::Node staticParams() const override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  bool isValid() override;
};
}  // namespace setup_assistant
}  // namespace gui
