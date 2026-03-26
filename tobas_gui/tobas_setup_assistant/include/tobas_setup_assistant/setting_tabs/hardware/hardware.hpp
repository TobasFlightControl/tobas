#pragma once

#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/stacked_widget.hpp>

#include "../base_setting.hpp"
#include "./base.hpp"
#include "./dshot.hpp"
#include "./pwm.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace hw
{
class HardwareWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = HardwareWidget;
  using super = BaseSettingWidget;

  static constexpr char kTypeKey[] = "hardware_type";

  static constexpr char kPwmLabel[] = "PWM";
  static constexpr char kDShotLabel[] = "DShot";

public:
  explicit HardwareWidget(const uadf::Model& uadf, const Signals& sig);

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  const PwmWidget* pwm() const;
  const DShotWidget* dshot() const;

  const char* fmuName() const;
  const char* hardwarePackage() const;

  /* [Hz] */
  int imuUpdateRate() const;
  /* [rad/s/√Hz] */
  double gyroNoiseDensity() const;
  /* [rad/s^2/√Hz] */
  double gyroRandomWalk() const;
  /* [s] */
  int gyroBiasCorrTime() const;
  /* [m/s^2/√Hz] */
  double accNoiseDensity() const;
  /* [m/s^3/√Hz] */
  double accRandomWalk() const;
  /* [s] */
  int accBiasCorrTime() const;

  /* [Hz] */
  int magUpdateRate() const;
  /* [G] */
  double magNoiseStddev() const;
  /* [G] */
  double magHardBiasNorm() const;

  /* [Hz] */
  int presUpdateRate() const;
  /* [Pa] */
  double presNoiseStddev() const;

  /* [Hz] */
  int gnssUpdateRate() const;
  /* [m] */
  double gnssHorizontalPositionAccuracy() const;
  /* [m] */
  double gnssVerticalPositionAccuracy() const;
  /* [m/s] */
  double gnssHorizontalVelocityStddev() const;
  /* [m/s] */
  double gnssVerticalVelocityStddev() const;

  int numPwmChannels() const;
  int numDShotChannels() const;

private:
  qt::ComboBox* type_;
  qt::StackedWidget* hardwares_;
  PwmWidget* pwm_;
  DShotWidget* dshot_;

  BaseHardwareWidget* selected();
  const BaseHardwareWidget* selected() const;

private Q_SLOTS:
  void setCurrentHardware(int index);
};
};  // namespace hw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
