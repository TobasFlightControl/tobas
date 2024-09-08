#include <filesystem>
#include <QPushButton>
#include <QFileDialog>

#include <tobas_linux/core.hpp>
#include <tobas_qt_tools/message.hpp>

#include "tobas_setup_assistant/setting_tabs/fixed_wing/aero_coefs.hpp"
#include "tobas_setup_assistant/setting_tabs/fixed_wing/vspaero_parser.hpp"
#include "tobas_setup_assistant/setting_tabs/fixed_wing/constants.hpp"

namespace gui
{
namespace setup_assistant
{
namespace fixed_wing
{
AerodynamicsCoefficientsWidget::AerodynamicsCoefficientsWidget(rclcpp::Node::SharedPtr node)
  : node_(node), property_client_(node)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  const auto cols = new QHBoxLayout();
  rows->addLayout(cols);

  const auto load_button = new QPushButton("Load VSPAERO Output");
  load_button->setFixedSize(kButtonWidth, kButtonHeight);
  connect(load_button, &QPushButton::clicked, this, &self::onLoadButtonClicked);
  rows->addWidget(load_button);

  cols->addStretch();

  form_ = new qt::FormLayout();
  rows->addLayout(form_);

  c_lift_0_ = new qt::DoubleSpinBox();
  c_lift_0_->setDecimals(fixed_wing::kStabilityCoefDecimals);
  c_lift_0_->setValue(0.2127);
  c_lift_0_->setSuffix(" [-]");
  form_->addRow(new QLabel("c_lift_0"), c_lift_0_);

  c_lift_alpha_ = new qt::DoubleSpinBox();
  c_lift_alpha_->setDecimals(fixed_wing::kStabilityCoefDecimals);
  c_lift_alpha_->setValue(10.806);
  c_lift_alpha_->setSuffix(" [/rad]");
  form_->addRow(new QLabel("c_lift_alpha"), c_lift_alpha_);

  c_drag_0_ = new qt::DoubleSpinBox();
  c_drag_0_->setDecimals(fixed_wing::kStabilityCoefDecimals);
  c_drag_0_->setValue(0.136);
  c_drag_0_->setSuffix(" [-]");
  form_->addRow(new QLabel("c_drag_0"), c_drag_0_);

  c_drag_alpha_ = new qt::DoubleSpinBox();
  c_drag_alpha_->setDecimals(fixed_wing::kStabilityCoefDecimals);
  c_drag_alpha_->setValue(0.6737);
  c_drag_alpha_->setSuffix(" [/rad]");
  form_->addRow(new QLabel("c_drag_alpha"), c_drag_alpha_);

  c_side_beta_ = new qt::DoubleSpinBox();
  c_side_beta_->setDecimals(fixed_wing::kStabilityCoefDecimals);
  c_side_beta_->setValue(-0.3073);
  c_side_beta_->setSuffix(" [/rad]");
  form_->addRow(new QLabel("c_side_beta"), c_side_beta_);

  c_roll_beta_ = new qt::DoubleSpinBox();
  c_roll_beta_->setDecimals(fixed_wing::kStabilityCoefDecimals);
  c_roll_beta_->setValue(-0.0154);
  c_roll_beta_->setSuffix(" [/rad]");
  form_->addRow(new QLabel("c_roll_beta"), c_roll_beta_);

  c_roll_p_ = new qt::DoubleSpinBox();
  c_roll_p_->setDecimals(fixed_wing::kStabilityCoefDecimals);
  c_roll_p_->setValue(-0.1647);
  c_roll_p_->setSuffix(" [s/rad]");
  form_->addRow(new QLabel("c_roll_p"), c_roll_p_);

  c_roll_r_ = new qt::DoubleSpinBox();
  c_roll_r_->setDecimals(fixed_wing::kStabilityCoefDecimals);
  c_roll_r_->setValue(0.0117);
  c_roll_r_->setSuffix(" [s/rad]");
  form_->addRow(new QLabel("c_roll_r"), c_roll_r_);

  c_pitch_0_ = new qt::DoubleSpinBox();
  c_pitch_0_->setDecimals(fixed_wing::kStabilityCoefDecimals);
  c_pitch_0_->setValue(0.0435);
  c_pitch_0_->setSuffix(" [-]");
  form_->addRow(new QLabel("c_pitch_0"), c_pitch_0_);

  c_pitch_alpha_ = new qt::DoubleSpinBox();
  c_pitch_alpha_->setDecimals(fixed_wing::kStabilityCoefDecimals);
  c_pitch_alpha_->setValue(-2.969);
  c_pitch_alpha_->setSuffix(" [/rad]");
  form_->addRow(new QLabel("c_pitch_alpha"), c_pitch_alpha_);

  c_pitch_abs_beta_ = new qt::DoubleSpinBox();
  c_pitch_abs_beta_->setDecimals(fixed_wing::kStabilityCoefDecimals);
  c_pitch_abs_beta_->setSuffix(" [/rad]");
  c_pitch_abs_beta_->setValue(0.0);
  form_->addRow(new QLabel("c_pitch_abs_beta"), c_pitch_abs_beta_);

  c_pitch_alpha_rate_ = new qt::DoubleSpinBox();
  c_pitch_alpha_rate_->setDecimals(fixed_wing::kStabilityCoefDecimals);
  c_pitch_alpha_rate_->setSuffix(" [s/rad]");
  c_pitch_alpha_rate_->setValue(0.0);
  form_->addRow(new QLabel("c_pitch_alpha_rate"), c_pitch_alpha_rate_);

  c_pitch_q_ = new qt::DoubleSpinBox();
  c_pitch_q_->setDecimals(fixed_wing::kStabilityCoefDecimals);
  c_pitch_q_->setSuffix(" [s/rad]");
  c_pitch_q_->setValue(-106.1542);
  form_->addRow(new QLabel("c_pitch_q"), c_pitch_q_);

  c_yaw_beta_ = new qt::DoubleSpinBox();
  c_yaw_beta_->setDecimals(fixed_wing::kStabilityCoefDecimals);
  c_yaw_beta_->setSuffix(" [/rad]");
  c_yaw_beta_->setValue(0.043);
  form_->addRow(new QLabel("c_yaw_beta"), c_yaw_beta_);

  c_yaw_p_ = new qt::DoubleSpinBox();
  c_yaw_p_->setDecimals(fixed_wing::kStabilityCoefDecimals);
  c_yaw_p_->setSuffix(" [s/rad]");
  c_yaw_p_->setValue(0.0);
  form_->addRow(new QLabel("c_yaw_p"), c_yaw_p_);

  c_yaw_r_ = new qt::DoubleSpinBox();
  c_yaw_r_->setDecimals(fixed_wing::kStabilityCoefDecimals);
  c_yaw_r_->setSuffix(" [s/rad]");
  c_yaw_r_->setValue(-0.0827);
  form_->addRow(new QLabel("c_yaw_r"), c_yaw_r_);
}

void AerodynamicsCoefficientsWidget::updateInternalDataStructures()
{
}

bool AerodynamicsCoefficientsWidget::isValid()
{
  return true;
}

YAML::Node AerodynamicsCoefficientsWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  for (int row = 0; row < form_->rowCount(); ++row)
  {
    const auto label = qobject_cast<QLabel*>(form_->getLabel(row));
    const auto widget = qobject_cast<qt::DoubleSpinBox*>(form_->getWidget(row));
    node[label->text().toStdString()] = widget->value();
  }

  return node;
}

void AerodynamicsCoefficientsWidget::load(const YAML::Node& node)
{
  for (int row = 0; row < form_->rowCount(); ++row)
  {
    const auto label = qobject_cast<QLabel*>(form_->getLabel(row));
    const auto widget = qobject_cast<qt::DoubleSpinBox*>(form_->getWidget(row));
    widget->setValue(node[label->text().toStdString()].as<double>());
  }
}

double AerodynamicsCoefficientsWidget::c_lift_0() const
{
  return c_lift_0_->value();
}

double AerodynamicsCoefficientsWidget::c_lift_alpha() const
{
  return c_lift_alpha_->value();
}

double AerodynamicsCoefficientsWidget::c_drag_0() const
{
  return c_drag_0_->value();
}

double AerodynamicsCoefficientsWidget::c_drag_alpha() const
{
  return c_drag_alpha_->value();
}

double AerodynamicsCoefficientsWidget::c_side_beta() const
{
  return c_side_beta_->value();
}

double AerodynamicsCoefficientsWidget::c_roll_beta() const
{
  return c_roll_beta_->value();
}

double AerodynamicsCoefficientsWidget::c_roll_p() const
{
  return c_roll_p_->value();
}

double AerodynamicsCoefficientsWidget::c_roll_r() const
{
  return c_roll_r_->value();
}

double AerodynamicsCoefficientsWidget::c_pitch_0() const
{
  return c_pitch_0_->value();
}

double AerodynamicsCoefficientsWidget::c_pitch_alpha() const
{
  return c_pitch_alpha_->value();
}

double AerodynamicsCoefficientsWidget::c_pitch_abs_beta() const
{
  return c_pitch_abs_beta_->value();
}

double AerodynamicsCoefficientsWidget::c_pitch_alpha_rate() const
{
  return c_pitch_alpha_rate_->value();
}

double AerodynamicsCoefficientsWidget::c_pitch_q() const
{
  return c_pitch_q_->value();
}

double AerodynamicsCoefficientsWidget::c_yaw_beta() const
{
  return c_yaw_beta_->value();
}

double AerodynamicsCoefficientsWidget::c_yaw_p() const
{
  return c_yaw_p_->value();
}

double AerodynamicsCoefficientsWidget::c_yaw_r() const
{
  return c_yaw_r_->value();
}

void AerodynamicsCoefficientsWidget::onLoadButtonClicked()
{
  // 前回開いたパスを取得
  std::string last_opened_dir;
  if (property_client_.get(kLastOpenedDirKey, last_opened_dir) < 0)
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), property_client_.errorMessage());
    last_opened_dir = linux::homeDir();
  }

  // paramsのパスを取得
  const auto options = QFileDialog::DontUseNativeDialog;
  const auto file_path = QFileDialog::getOpenFileName(
    this, kTitle, QString::fromStdString(last_opened_dir), "OpenVSP Stability Derivatives (*.stab)", nullptr, options);

  // キャンセルの場合は何もせずに終了
  if (file_path.isEmpty())
    return;

  // ユーザが開いたディレクトリを保存
  const auto par_dir = std::filesystem::path(file_path.toStdString()).parent_path();
  if (property_client_.set(kLastOpenedDirKey, par_dir) < 0)
    RCLCPP_WARN_STREAM(node_->get_logger(), property_client_.errorMessage());
  if (property_client_.save() < 0)
    RCLCPP_WARN_STREAM(node_->get_logger(), property_client_.errorMessage());

  // パラメータを読み込む
  VSPAEROParser parser;
  if (!parser.parse(file_path.toStdString()))
  {
    qt::qErrorBox(this, "Failed to read coefficients.");
    return;
  }

  // 読み込んだパラメータをフォームに反映
  c_lift_0_->setValue(parser.c_lift_0());
  c_lift_alpha_->setValue(parser.c_lift_alpha());
  c_drag_0_->setValue(parser.c_drag_0());
  c_drag_alpha_->setValue(parser.c_drag_alpha());
  c_side_beta_->setValue(parser.c_side_beta());
  c_roll_beta_->setValue(parser.c_roll_beta());
  c_roll_p_->setValue(parser.c_roll_p());
  c_roll_r_->setValue(parser.c_roll_r());
  c_pitch_0_->setValue(parser.c_pitch_0());
  c_pitch_alpha_->setValue(parser.c_pitch_alpha());
  c_pitch_abs_beta_->setValue(parser.c_pitch_abs_beta());
  c_pitch_alpha_rate_->setValue(parser.c_pitch_alpha_rate());
  c_pitch_q_->setValue(parser.c_pitch_q());
  c_yaw_beta_->setValue(parser.c_yaw_beta());
  c_yaw_p_->setValue(parser.c_yaw_p());
  c_yaw_r_->setValue(parser.c_yaw_r());

  qt::qInfoBox(this, "Coefficients are loaded successfully.");
}
}  // namespace fixed_wing
}  // namespace setup_assistant
}  // namespace gui
