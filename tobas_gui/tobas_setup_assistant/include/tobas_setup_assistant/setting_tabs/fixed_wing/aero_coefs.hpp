#pragma once

#include <tobas_property_client/property_client.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>

#include "tobas_setup_assistant/param_getters/double_spin_box.hpp"
#include "./base.hpp"

namespace gui
{
namespace sa
{
namespace fixed_wing
{
class AerodynamicsCoefficientsWidget : public BaseSelectedLinkSettingWidget
{
  Q_OBJECT

  using self = AerodynamicsCoefficientsWidget;
  using super = BaseSelectedLinkSettingWidget;

  static constexpr int kButtonWidth = 180;
  static constexpr int kButtonHeight = 60;
  static constexpr char kLastOpenedDirKey[] = "aerodynamics/last_opened_dir";

public:
  explicit AerodynamicsCoefficientsWidget(rclcpp::Node::SharedPtr node);

  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

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

  qt::FormLayout* form_;

  qt::DoubleSpinBox* c_lift_0_;
  qt::DoubleSpinBox* c_lift_alpha_;
  qt::DoubleSpinBox* c_drag_0_;
  qt::DoubleSpinBox* c_drag_alpha_;
  qt::DoubleSpinBox* c_side_beta_;
  qt::DoubleSpinBox* c_roll_beta_;
  qt::DoubleSpinBox* c_roll_p_;
  qt::DoubleSpinBox* c_roll_r_;
  qt::DoubleSpinBox* c_pitch_0_;
  qt::DoubleSpinBox* c_pitch_alpha_;
  qt::DoubleSpinBox* c_pitch_abs_beta_;
  qt::DoubleSpinBox* c_pitch_alpha_rate_;
  qt::DoubleSpinBox* c_pitch_q_;
  qt::DoubleSpinBox* c_yaw_beta_;
  qt::DoubleSpinBox* c_yaw_p_;
  qt::DoubleSpinBox* c_yaw_r_;
};
}  // namespace fixed_wing
}  // namespace sa
}  // namespace gui
