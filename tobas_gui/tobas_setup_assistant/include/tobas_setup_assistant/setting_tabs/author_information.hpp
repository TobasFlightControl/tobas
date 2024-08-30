#pragma once

#include "./base_setting.hpp"
#include "../param_getters/line_edit.hpp"

namespace gui
{
namespace setup_assistant
{
class AuthorInformationWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = AuthorInformationWidget;
  using super = BaseSettingWidget;

public:
  using super::BaseSettingWidget;

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void onInit() override;
  void onOpened() override;
  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() override;
  void load(const YAML::Node& node) override;

private:
  ParamGetterWidget_LineEdit* name_;
  ParamGetterWidget_LineEdit* email_;
};
};  // namespace setup_assistant
}  // namespace gui
