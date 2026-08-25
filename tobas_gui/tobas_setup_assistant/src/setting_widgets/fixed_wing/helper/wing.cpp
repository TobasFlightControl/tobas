#include <tobas_setup_assistant/setting_tabs/fixed_wing/helper/wing.hpp>

#include <QLabel>
#include <QVBoxLayout>

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_yaml_tools/convert/eigen.hpp>
#include <tobas_yaml_tools/format.hpp>

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

  const auto form = new qt::FormLayout();
  rows->addLayout(form);

  const auto symmetric_label = new QLabel("Symmetric");
  symmetric_label->setToolTip("If 'Symmetric' is unchecked, the right side of the wing is treated as existing.");
  symmetric_ = new QCheckBox();
  form->addRow(symmetric_label, symmetric_);

  c_root_ = new qt::DoubleSpinBox();
  c_root_->setDecimals(3);
  c_root_->setMinimum(1e-3);
  c_root_->setSuffix(" m");
  form->addRow(new QLabel("Root Chord Length"), c_root_);

  c_tip_ = new qt::DoubleSpinBox();
  c_tip_->setDecimals(3);
  c_tip_->setMinimum(1e-3);
  c_tip_->setSuffix(" m");
  form->addRow(new QLabel("Tip Chord Length"), c_tip_);

  span_ = new qt::DoubleSpinBox();
  span_->setDecimals(3);
  span_->setMinimum(1e-3);
  span_->setSuffix(" m");
  const auto span_label = new QLabel("Span Length");
  span_label->setToolTip("If 'Symmetric' is unchecked, consider the distance from the wing root to the wing tip to be b/2.");
  form->addRow(span_label, span_);

  oswald_efficiency_ = new qt::DoubleSpinBox();
  oswald_efficiency_->setDecimals(3);
  oswald_efficiency_->setMinimum(1e-3);
  oswald_efficiency_->setMaximum(1.0);
  oswald_efficiency_->setSuffix("");
  form->addRow(new QLabel("Oswald Efficiency"), oswald_efficiency_);

  c_lift_0_ = new qt::DoubleSpinBox();
  c_lift_0_->setDecimals(3);
  c_lift_0_->setSuffix("");
  form->addRow(new QLabel("C_lift_0"), c_lift_0_);

  c_drag_0_ = new qt::DoubleSpinBox();
  c_drag_0_->setDecimals(3);
  c_drag_0_->setSuffix("");
  form->addRow("C_drag_0", c_drag_0_);

  c_pitch_0_ = new qt::DoubleSpinBox();
  c_pitch_0_->setDecimals(3);
  c_pitch_0_->setSuffix("");
  form->addRow(new QLabel("C_pitch_0"), c_pitch_0_);

  sweep_back_ = new qt::DoubleSpinBox();
  sweep_back_->setDecimals(3);
  sweep_back_->setSuffix(" rad");
  form->addRow(new QLabel("Sweep Back Angle"), sweep_back_);

  position_ = new qt::Vector3dEditHorizontal();
  position_->setDecimals(3);
  position_->setSuffix(" m");
  const auto position_label = new QLabel("Position");
  position_label->setToolTip("The position of the leading edge of the main wingtip as viewed in the base_link coordinate frame.");
  form->addVAlignedRow(position_label, position_);

  rotation_ = new qt::Vector3dEditHorizontal();
  rotation_->setDecimals(3);
  rotation_->setSuffix(" rad");
  const auto rotation_label = new QLabel("Rotation");
  rotation_label->setToolTip(
    "The rotation of the main wing coordinate frame as viewed in the base_link coordinate frame, expressed as Euler "
    "angles for rotations performed in the order X-Y-Z.");
  form->addVAlignedRow(rotation_label, rotation_);
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
  position_->setVector(Eigen::Vector3d::Zero(3));
  rotation_->setVector(Eigen::Vector3d::Zero(3));
}

bool WingWidget::isValid() const
{
  return true;
}

YAML::Node WingWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kSymmetricLabel] = symmetric();
  node[kCRootLabel] = yaml::format(c_root());
  node[kCTipLabel] = yaml::format(c_tip());
  node[kSpanLabel] = yaml::format(span());
  node[kOswaldLabel] = yaml::format(oswaldEfficiency());
  node[kCLift0Label] = yaml::format(c_lift_0());
  node[kCDrag0Label] = yaml::format(c_drag_0());
  node[kCPitch0Label] = yaml::format(c_pitch_0());
  node[kSweepBackLabel] = yaml::format(sweepBack());
  node[kPositionLabel] = position();
  node[kRotationLabel] = rotation();

  return node;
}

void WingWidget::load(const YAML::Node& node)
{
  symmetric(node[kSymmetricLabel].as<bool>());
  c_root(node[kCRootLabel].as<double>());
  c_tip(node[kCTipLabel].as<double>());
  span(node[kSpanLabel].as<double>());
  oswaldEfficiency(node[kOswaldLabel].as<double>());
  c_lift_0(node[kCLift0Label].as<double>());
  c_drag_0(node[kCDrag0Label].as<double>());
  c_pitch_0(node[kCPitch0Label].as<double>());
  sweepBack(node[kSweepBackLabel].as<double>());
  position(node[kPositionLabel].as<Eigen::Vector3d>());
  rotation(node[kRotationLabel].as<Eigen::Vector3d>());
}

bool WingWidget::symmetric() const
{
  return symmetric_->isChecked();
}

double WingWidget::c_root() const
{
  return c_root_->value();
}

double WingWidget::c_tip() const
{
  return c_tip_->value();
}

double WingWidget::span() const
{
  return span_->value();
}

double WingWidget::oswaldEfficiency() const
{
  return oswald_efficiency_->value();
}

double WingWidget::c_lift_0() const
{
  return c_lift_0_->value();
}

double WingWidget::c_drag_0() const
{
  return c_drag_0_->value();
}

double WingWidget::c_pitch_0() const
{
  return c_pitch_0_->value();
}

double WingWidget::sweepBack() const
{
  return sweep_back_->value();
}

Eigen::Vector3d WingWidget::position() const
{
  return position_->vector();
}

Eigen::Vector3d WingWidget::rotation() const
{
  return rotation_->vector();
}

void WingWidget::symmetric(const bool& value)
{
  symmetric_->setChecked(value);
}

void WingWidget::c_root(const double& value)
{
  c_root_->setValue(value);
}

void WingWidget::c_tip(const double& value)
{
  c_tip_->setValue(value);
}

void WingWidget::span(const double& value)
{
  span_->setValue(value);
}

void WingWidget::oswaldEfficiency(const double& value)
{
  oswald_efficiency_->setValue(value);
}

void WingWidget::c_lift_0(const double& value)
{
  c_lift_0_->setValue(value);
}

void WingWidget::c_drag_0(const double& value)
{
  c_drag_0_->setValue(value);
}

void WingWidget::c_pitch_0(const double& value)
{
  c_pitch_0_->setValue(value);
}

void WingWidget::sweepBack(const double& value)
{
  sweep_back_->setValue(value);
}

void WingWidget::position(const Eigen::Vector3d& value)
{
  position_->setVector(value);
}

void WingWidget::rotation(const Eigen::Vector3d& value)
{
  rotation_->setVector(value);
}

double WingWidget::surfaceArea() const
{
  if (symmetric()) {
    return 0.5 * (c_root() + c_tip()) * span();
  }
  else {
    return 0.25 * (c_root() + c_tip()) * span();
  }
}

double WingWidget::aspectRatio() const
{
  return span() * span() / surfaceArea();  // 非対称だと怪しいが翼根が壁面についている場合ならこれが正解
}

double WingWidget::c_mac() const
{
  const auto lambda = c_tip() / c_root();
  return (2.0 / 3.0) * (lambda * lambda + lambda + 1) / (lambda + 1) * c_root();
}

double WingWidget::y_mac() const
{
  const auto lambda = c_tip() / c_root();
  return (lambda + 2) / (2 * lambda + 1) * 0.5 * span();
}

Eigen::Vector3d WingWidget::mac_position() const
{
  const auto x_mac = -(0.25 * c_root() + y_mac() * tan(sweepBack()));
  if (symmetric()) {
    return Eigen::Vector3d(x_mac, 0, 0);
  }
  else {
    // symmetricでない場合は右側が残るので, flu座標系でみてy軸方向負の部分が残っている
    return Eigen::Vector3d(x_mac, -y_mac(), 0);
  }
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
  return -y_mac() / span() * c_lift(alpha) * tan(sweepBack());
}

double WingWidget::c_roll_p() const
{
  return -c_lift_alpha() / (8 * 0.5 * (c_root() + c_tip())) * (c_root() / 3.0 + c_tip());
}

double WingWidget::c_roll_r(const double& alpha) const
{
  return (8.0 * c_lift(alpha)) / (0.5 * span() * (c_root() + c_tip()) * pow(span(), 2)) *
         (1.0 / 12.0 * c_root() + 1.0 / 4.0 * c_tip()) * pow(0.5 * span(), 3);
}

double
WingWidget::c_roll(const double& V, const double& alpha, const double& beta, const double& p, const double& r) const
{
  return c_roll_beta(alpha) * beta + c_roll_p() * span() / (2.0 * V) * p + c_roll_r(alpha) * span() / (2.0 * V) * r;
}

double WingWidget::c_yaw_p(const double& alpha) const
{
  return -0.5 * (1 - 2.0 * c_lift_alpha() / (M_PI * oswaldEfficiency() * aspectRatio())) * c_roll_r(alpha);
}

double WingWidget::c_yaw_r(const double& alpha) const
{
  return -(8.0 * c_drag(alpha)) / (0.5 * span() * (c_root() + c_tip()) * pow(span(), 2)) *
         (1.0 / 12.0 * c_root() + 1.0 / 4.0 * c_tip()) * pow(0.5 * span(), 3);
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
