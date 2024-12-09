#pragma once

#include "tobas_setup_assistant/robot_info.hpp"
#include "../imu.hpp"
#include "../barometer.hpp"
#include "../gps.hpp"
#include "./base.hpp"

namespace gui
{
namespace setup_assistant
{
class ErrorStateKalmanFilterWidget : public BaseObserverWidget
{
  Q_OBJECT

public:
  explicit ErrorStateKalmanFilterWidget(
    const RobotInfo& robot,
    const IMUWidget* imu,
    const BarometerWidget* baro,
    const GPSWidget* gps);

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
  const IMUWidget* imu_;
  const BarometerWidget* baro_;
  const GPSWidget* gps_;

  QCheckBox* do_acc_bias_estimation_;
  QCheckBox* do_gyro_bias_estimation_;
  QCheckBox* do_grav_estimation_;
};
}  // namespace setup_assistant
}  // namespace gui
