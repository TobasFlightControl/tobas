#pragma once

#include "../barometer.hpp"
#include "../gnss.hpp"
#include "../imu.hpp"
#include "./base.hpp"
#include "tobas_setup_assistant/robot_info.hpp"

namespace gui
{
namespace sa
{
class ErrorStateKalmanFilterWidget : public BaseObserverWidget
{
  Q_OBJECT

public:
  explicit ErrorStateKalmanFilterWidget(
    const RobotInfo& robot,
    const ImuWidget* imu,
    const BarometerWidget* baro,
    const GnssWidget* gnss);

  const char* name() const override;
  const char* description() const override;
  QString observerPackage() const override;
  QString pluginName() const override;

  YAML::Node staticParams() const override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  bool isValid() override;

private:
  const RobotInfo& robot_;
  const ImuWidget* imu_;
  const BarometerWidget* baro_;
  const GnssWidget* gnss_;

  QCheckBox* do_acc_bias_estimation_;
  QCheckBox* do_gyro_bias_estimation_;
  QCheckBox* do_mag_hard_bias_estimation_;
  QCheckBox* do_mag_soft_bias_estimation_;
  QCheckBox* do_grav_estimation_;
};
}  // namespace sa
}  // namespace gui
