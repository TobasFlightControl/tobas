#pragma once

#include <QWidget>

#include "tobas_setup_assistant/param_getters/double_spin_box.hpp"
#include "tobas_setup_assistant/param_getters/vector3d.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace fw
{
class WingWidget : public QWidget
{
  Q_OBJECT

  using self = WingWidget;
  using super = QWidget;

public:
  explicit WingWidget();
  void updateInternalDataStructures();
  void setToDefaults();
  bool isValid();

private:
  ParamGetterWidget_DoubleSpinBox* c_root_;
  ParamGetterWidget_DoubleSpinBox* c_tip_;
  ParamGetterWidget_DoubleSpinBox* span_;
  ParamGetterWidget_DoubleSpinBox* c_lift_0_;
};
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
