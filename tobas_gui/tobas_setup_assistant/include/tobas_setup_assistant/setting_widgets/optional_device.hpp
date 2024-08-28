#pragma once

#include <QCheckBox>

#include "./base_setting.hpp"

namespace gui
{
namespace setup_assistant
{
class OptionalDeviceWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = OptionalDeviceWidget;
  using super = BaseSettingWidget;

public:
  using super::BaseSettingWidget;

  void initialize() override;

protected:
  virtual bool defaultEquipped() const = 0;

  /* Equippedがチェックされているときだけ有効になるウィジェットを追加する． */
  template <typename T>
  void addParamWidget(ParamGetterWidget<T>* widget);

private Q_SLOTS:
  void onEquippedToggled(bool checked);

private:
  QCheckBox* equipped_;
  QWidget* config_;
  QVBoxLayout* param_rows_;

  bool equipped() const;
};

template <typename T>
void OptionalDeviceWidget::addParamWidget(ParamGetterWidget<T>* widget)
{
  param_rows_->addWidget(widget);
}
};  // namespace setup_assistant
}  // namespace gui
