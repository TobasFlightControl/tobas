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

protected:
  static constexpr char kEquippedKey[] = "equipped";

public:
  explicit OptionalDeviceWidget();

  bool equipped() const;

protected:
  QCheckBox* equipped_;

  virtual bool defaultEquipped() const = 0;

  /* Equippedがチェックされているときだけ有効になるウィジェットを追加する． */
  template <typename T>
  void addParamWidget(ParamGetterWidget<T>* widget);

private:
  QWidget* config_;
  QVBoxLayout* param_rows_;

private Q_SLOTS:
  void initialize();
  void onEquippedToggled(bool checked);
};

template <typename T>
void OptionalDeviceWidget::addParamWidget(ParamGetterWidget<T>* widget)
{
  param_rows_->addWidget(widget);
}
};  // namespace setup_assistant
}  // namespace gui
