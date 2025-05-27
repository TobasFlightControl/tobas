#pragma once

#include "../../param_getters/line_edit.hpp"
#include "./base.hpp"

namespace gui
{
namespace sa
{
class CustomObserverWidget : public BaseObserverWidget
{
  Q_OBJECT

public:
  explicit CustomObserverWidget();

  const char* name() const override;
  const char* description() const override;
  QString observerPackage() const override;
  QString pluginName() const override;

  YAML::Node staticParams() const override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  bool isValid() override;

private:
  ParamGetterWidget_LineEdit* package_;
  ParamGetterWidget_LineEdit* plugin_;
};
}  // namespace sa
}  // namespace gui
