#pragma once

#include <QCheckBox>
#include <QVBoxLayout>

#include "../base_setting.hpp"
#include "./vehicle.hpp"
#include "./aero_coefs.hpp"
#include "./control_surface/control_surfaces.hpp"

namespace gui
{
namespace setup_assistant
{
namespace fixed_wing
{
class FixedWingWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = FixedWingWidget;
  using super = BaseSettingWidget;

  static constexpr char kVehicleLabel[] = "Vehicle Parameters";
  static constexpr char kAeroCoefsLabel[] = "Aerodynamic Coefficients";
  static constexpr char kControlSurfacesLabel[] = "Control Surfaces";

  static constexpr bool kDefaultHasFixedWing = false;

public:
  explicit FixedWingWidget(rclcpp::Node::SharedPtr node, const RobotInfo& robot);

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void onInit() override;
  void onOpened() override;
  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() override;
  void load(const YAML::Node& node) override;

  bool hasFixedWing() const;
  const VehicleParametersWidget* vehicle() const;
  const AerodynamicsCoefficientsWidget* aeroCoefs() const;
  const ControlSurfacesWidget* controlSurfaces() const;

private Q_SLOTS:
  void setSettingWidgetsEnabled(bool enabled);

private:
  const rclcpp::Node::SharedPtr node_;
  const RobotInfo& robot_;

  QCheckBox* has_fixed_wing_;
  QVBoxLayout* setting_rows_;

  VehicleParametersWidget* vehicle_;
  AerodynamicsCoefficientsWidget* aero_coefs_;
  ControlSurfacesWidget* control_surfaces_;
};
};  // namespace fixed_wing
}  // namespace setup_assistant
}  // namespace gui
