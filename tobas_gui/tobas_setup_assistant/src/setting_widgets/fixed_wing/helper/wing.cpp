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
namespace hp
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

  oswald_efficiency_ = new ParamGetterWidget_DoubleSpinBox("Oswald Efficiency", "");
  oswald_efficiency_->setDecimals(3);
  oswald_efficiency_->setMinimum(1e-3);
  oswald_efficiency_->setMaximum(1.0);
  oswald_efficiency_->setSuffix("");
  rows->addWidget(oswald_efficiency_);

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
  oswald_efficiency_->setValue(0.8);
  c_lift_0_->setValue(0.0);
  c_drag_0_->setValue(0.0);
  c_pitch_0_->setValue(0.0);
  sweep_back_->setValue(0.0);
  position_->setValue(Eigen::Vector3d::Zero(3));
  rotation_->setValue(Eigen::Vector3d::Zero(3));
}

bool WingWidget::isValid() const
{
  return true;
}

bool WingWidget::symmetric() const
{
  return symmetric_->isChecked();
}

double WingWidget::c_root() const
{
  return c_root_->getValue();
}

double WingWidget::c_tip() const
{
  return c_tip_->getValue();
}

double WingWidget::span() const
{
  return span_->getValue();
}

double WingWidget::oswaldEfficiency() const
{
  return oswald_efficiency_->getValue();
}

double WingWidget::c_lift_0() const
{
  return c_lift_0_->getValue();
}

double WingWidget::c_drag_0() const
{
  return c_drag_0_->getValue();
}

double WingWidget::c_pitch_0() const
{
  return c_pitch_0_->getValue();
}

double WingWidget::sweepBack() const
{
  return sweep_back_->getValue();
}

Eigen::Vector3d WingWidget::position() const
{
  return position_->getValue();
}

Eigen::Vector3d WingWidget::rotation() const
{
  return rotation_->getValue();
}

double WingWidget::surfaceArea() const
{
  if (symmetric()) {
    return 0.5 * (c_root() + c_tip()) * span();
  } else {
    return 0.25 * (c_root() + c_tip()) * span();
  }
}

double WingWidget::aspectRatio() const
{
  return span() * span() / surfaceArea(); // 非対称だと怪しいが翼根が壁面についている場合ならこれが正解
}

double WingWidget::c_mac() const
{
  const auto lambda = c_tip() / c_root();
  return (2.0 / 3.0) * (lambda* lambda + lambda + 1) / (lambda + 1) * c_root();
}

double WingWidget::y_mac() const
{
  const auto lambda = c_tip() / c_root();
  return (lambda + 2) / (2 * lambda + 1) * 0.5 * span();
}

double WingWidget::c_lift_alpha() const
{
  return 2.0 * M_PI / (1 + 2.0 / (oswaldEfficiency() * M_PI * aspectRatio())) * cos(sweepBack());
}

double WingWidget::c_lift(const double& alpha) const
{
  return c_lift_0() + c_lift_alpha() * alpha;
}

double WingWidget::c_drag(const double& alpha) const
{
  const auto c_l = c_lift(alpha);
  return c_drag_0() + c_l * c_l / (oswaldEfficiency() * M_PI * aspectRatio());
}

double WingWidget::c_roll_beta(const double& alpha) const
{
  return - y_mac() / span() * c_lift(alpha) * tan(sweepBack());
}

double WingWidget::c_roll_p() const
{
  return - c_lift_alpha() / (8 * 0.5 * (c_root() + c_tip())) * (c_root() / 3.0 + c_tip());
}

double WingWidget::c_roll_r(const double& alpha) const
{
  return (8.0 * c_lift(alpha)) / (0.5 * span() * (c_root() + c_tip()) * pow(span(), 2)) * (1.0 / 12.0 * c_root() + 1.0 / 4.0 * c_tip()) * pow(0.5 * span(), 3);
}

double WingWidget::c_roll(const double& V, const double& alpha, const double& beta, const double& p, const double& r) const
{
  return c_roll_beta(alpha) * beta + c_roll_p() * span() / (2.0 * V) * p + c_roll_r(alpha) * span() / (2.0 * V) * r;
}

double WingWidget::c_yaw_p(const double& alpha) const
{
  return -0.5 * (1 - 2.0 * c_lift_alpha() / (M_PI * oswaldEfficiency() * aspectRatio())) * c_roll_r(alpha);
}

double WingWidget::c_yaw_r(const double& alpha) const
{
  return - (8.0 * c_drag(alpha)) / (0.5 * span() * (c_root() + c_tip()) * pow(span(), 2)) * (1.0 / 12.0 * c_root() + 1.0 / 4.0 * c_tip()) * pow(0.5 * span(), 3);
}

double WingWidget::c_yaw(const double& V, const double& alpha, const double& p, const double& r) const
{
  return c_yaw_p(alpha) * span() / (2.0 * V) * p + c_yaw_r(alpha) * span() / (2.0 * V) * r;
}
}  // namespace hp
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
