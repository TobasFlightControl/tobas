#include <tobas_setup_assistant/setting_tabs/fixed_wing/helper/wing.hpp>

#include <QVBoxLayout>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace fw
{
WingWidget::WingWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  c_root_ = new ParamGetterWidget_DoubleSpinBox("Root Chord Length", "");
  c_root_->setDecimals(3);
  c_root_->setMinimum(1e-3);
  c_root_->setSuffix("m");
  rows->addWidget(c_root_);

  c_tip_ = new ParamGetterWidget_DoubleSpinBox("Tip Chord Length", "");
  c_tip_->setDecimals(3);
  c_tip_->setMinimum(1e-3);
  c_tip_->setSuffix("m");
  rows->addWidget(c_tip_);

  span_ = new ParamGetterWidget_DoubleSpinBox("Span Length", "If “half” is checked, consider the distance from the wing root to the wing tip to be b/2.");
  span_->setDecimals(3);
  span_->setMinimum(1e-3);
  span_->setSuffix("m");
  rows->addWidget(span_);

  c_lift_0_ = new ParamGetterWidget_DoubleSpinBox("C_lift_0", "");
  c_lift_0_->setDecimals(3);
  c_lift_0_->setSuffix("");
  rows->addWidget(c_lift_0_);
}

void WingWidget::updateInternalDataStructures()
{
}

void WingWidget::setToDefaults()
{
  c_root_->setValue(0.3);
  c_tip_->setValue(0.2);
  span_->setValue(2.0);
  c_lift_0_->setValue(0.000);
}

bool WingWidget::isValid()
{
  return true;
}

}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
