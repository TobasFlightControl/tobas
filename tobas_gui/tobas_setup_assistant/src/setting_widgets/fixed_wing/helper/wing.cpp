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

  const auto header = new QHBoxLayout();
  header->setContentsMargins(rows->contentsMargins()); // ParamGetterWidgetと文字の左端を合わせるための苦肉の策
  const auto label = new QLabel("Symmetric");
  label->setFont(qt::DefaultFont(cmn::kLabelPSize, QFont::Bold));
  label->setAlignment(Qt::AlignTop | Qt::AlignLeft);
  symmetric_ = new QCheckBox();
  symmetric_->setFont(qt::DefaultFont(cmn::kLabelPSize, QFont::Bold));
  header->addWidget(label);
  header->addWidget(symmetric_);
  header->addStretch();
  rows->addLayout(header);

  c_root_ = new ParamGetterWidget_DoubleSpinBox("Root Chord Length", "");
  c_root_->setDecimals(3);
  c_root_->setMinimum(1e-3);
  c_root_->setSuffix(" m");
  rows->addWidget(c_root_);

  c_tip_ = new ParamGetterWidget_DoubleSpinBox("Tip Chord Length", "");
  c_tip_->setDecimals(3);
  c_tip_->setMinimum(1e-3);
  c_tip_->setSuffix(" m");
  rows->addWidget(c_tip_);

  span_ = new ParamGetterWidget_DoubleSpinBox("Span Length", "If “half” is checked, consider the distance from the wing root to the wing tip to be b/2.");
  span_->setDecimals(3);
  span_->setMinimum(1e-3);
  span_->setSuffix(" m");
  rows->addWidget(span_);

  c_lift_0_ = new ParamGetterWidget_DoubleSpinBox("C_lift_0", "");
  c_lift_0_->setDecimals(3);
  c_lift_0_->setSuffix("");
  rows->addWidget(c_lift_0_);

  c_drag_0_ = new ParamGetterWidget_DoubleSpinBox("C_drag_0", "");
  c_drag_0_->setDecimals(3);
  c_drag_0_->setSuffix("");
  rows->addWidget(c_drag_0_);

  c_pitch_0_ = new ParamGetterWidget_DoubleSpinBox("C_pitch_0", "");
  c_pitch_0_->setDecimals(3);
  c_pitch_0_->setSuffix("");
  rows->addWidget(c_pitch_0_);

  sweep_back_ = new ParamGetterWidget_DoubleSpinBox("Sweep Back Angle", "");
  sweep_back_->setDecimals(3);
  sweep_back_->setSuffix(" rad");
  rows->addWidget(sweep_back_);

  position_ = new ParamGetterWidget_Vector3d("Position", "The position of the leading edge of the main wingtip as viewed in the base_link coordinate frame.");
  position_->setDecimals(3);
  position_->setSuffix(" m");
  rows->addWidget(position_);

  rotation_ = new ParamGetterWidget_Vector3d("Rotation", "The rotation of the main wing coordinate frame as viewed in the base_link coordinate frame, expressed as Euler angles for rotations performed in the order X-Y-Z.");
  rotation_->setDecimals(3);
  rotation_->setSuffix(" rad");
  rows->addWidget(rotation_);
}

void WingWidget::updateInternalDataStructures()
{
}

void WingWidget::setToDefaults()
{
  symmetric_->setChecked(true);
  c_root_->setValue(0.3);
  c_tip_->setValue(0.2);
  span_->setValue(2.0);
  c_lift_0_->setValue(0.0);
  c_drag_0_->setValue(0.0);
  c_pitch_0_->setValue(0.0);
  sweep_back_->setValue(0.0);
  position_->setValue(Eigen::Vector3d::Zero(3));
  rotation_->setValue(Eigen::Vector3d::Zero(3));
}

bool WingWidget::isValid()
{
  return true;
}

bool WingWidget::symmetric()
{
  return symmetric_->isChecked();
}

double WingWidget::c_root()
{
  return c_root_->getValue();
}

double WingWidget::c_tip()
{
  return c_tip_->getValue();
}

double WingWidget::span()
{
  return span_->getValue();
}

double WingWidget::c_lift_0()
{
  return c_lift_0_->getValue();
}

double WingWidget::c_drag_0()
{
  return c_drag_0_->getValue();
}

double WingWidget::c_pitch_0()
{
  return c_pitch_0_->getValue();
}

double WingWidget::sweep_back()
{
  return sweep_back_->getValue();
}

Eigen::Vector3d WingWidget::position()
{
  return position_->getValue();
}

Eigen::Vector3d WingWidget::rotation()
{
  return rotation_->getValue();
}

}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
