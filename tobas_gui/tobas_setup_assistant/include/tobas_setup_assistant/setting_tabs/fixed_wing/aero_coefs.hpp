#pragma once

#include <yaml-cpp/yaml.h>

#include <tobas_property_client/property_client.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>

#include "tobas_setup_assistant/param_getters/double_spin_box.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace fw
{
class AerodynamicsCoefficientsWidget : public QWidget
{
  Q_OBJECT

  using self = AerodynamicsCoefficientsWidget;
  using super = QWidget;

  static constexpr int kButtonWidth = 180;
  static constexpr int kButtonHeight = 60;
  static constexpr char kLastOpenedDirKey[] = "last_opened_dir";

public:
  explicit AerodynamicsCoefficientsWidget(rclcpp::Node::SharedPtr node);

  void updateInternalDataStructures();
  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  double c_lift_0() const;
  double c_lift_alpha() const;
  double c_drag_0() const;
  double c_drag_alpha() const;
  double c_side_beta() const;
  double c_roll_beta() const;
  double c_roll_p() const;
  double c_roll_r() const;
  double c_pitch_0() const;
  double c_pitch_alpha() const;
  double c_pitch_abs_beta() const;
  double c_pitch_alpha_rate() const;
  double c_pitch_q() const;
  double c_yaw_beta() const;
  double c_yaw_p() const;
  double c_yaw_r() const;

private Q_SLOTS:
  void onLoadButtonClicked();

private:
  const rclcpp::Node::SharedPtr node_;
  ptree::PropertyClient property_client_;

  tobas::qt::FormLayout* form_;

  tobas::qt::DoubleSpinBox* c_lift_0_;
  tobas::qt::DoubleSpinBox* c_lift_alpha_;
  tobas::qt::DoubleSpinBox* c_drag_0_;
  tobas::qt::DoubleSpinBox* c_drag_alpha_;
  tobas::qt::DoubleSpinBox* c_side_beta_;
  tobas::qt::DoubleSpinBox* c_roll_beta_;
  tobas::qt::DoubleSpinBox* c_roll_p_;
  tobas::qt::DoubleSpinBox* c_roll_r_;
  tobas::qt::DoubleSpinBox* c_pitch_0_;
  tobas::qt::DoubleSpinBox* c_pitch_alpha_;
  tobas::qt::DoubleSpinBox* c_pitch_abs_beta_;
  tobas::qt::DoubleSpinBox* c_pitch_alpha_rate_;
  tobas::qt::DoubleSpinBox* c_pitch_q_;
  tobas::qt::DoubleSpinBox* c_yaw_beta_;
  tobas::qt::DoubleSpinBox* c_yaw_p_;
  tobas::qt::DoubleSpinBox* c_yaw_r_;
};
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
