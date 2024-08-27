#pragma once

#include <QCheckBox>

#include "./base.hpp"

namespace gui
{
namespace setup_assistant
{
class ParamGetterWidget_CheckBox : public ParamGetterWidget<bool>
{
  Q_OBJECT

  using self = ParamGetterWidget_CheckBox;
  using super = ParamGetterWidget<bool>;

Q_SIGNALS:
  void toggled(bool checked);

public:
  explicit ParamGetterWidget_CheckBox(
    const QString& param_name,
    const QString& description_text = "",
    const QString& check_box_text = "",
    bool _default = false);

  bool get() const override;
  bool set(const bool& src) override;

private Q_SLOTS:
  void onToggled(bool checked);

private:
  QCheckBox* box_;
};
}  // namespace setup_assistant
}  // namespace gui
