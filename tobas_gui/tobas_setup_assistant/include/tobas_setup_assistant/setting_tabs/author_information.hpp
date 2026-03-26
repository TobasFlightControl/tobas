#pragma once

#include "../param_getters/line_edit.hpp"
#include "./base_setting.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
class AuthorInformationWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = AuthorInformationWidget;
  using super = BaseSettingWidget;

public:
  explicit AuthorInformationWidget();

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  QString authorName() const;
  QString authorEmail() const;

private:
  ParamGetterWidget_LineEdit* name_;
  ParamGetterWidget_LineEdit* email_;

  static QString getDefaultName();
  static QString getDefaultEmail();
};
};  // namespace sa
}  // namespace gui
}  // namespace tobas
