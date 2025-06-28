#pragma once

#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>
#include <tobas_qt_tools/widgets/stacked_widget.hpp>

#include "./barometer.hpp"
#include "./base_setting.hpp"
#include "./gnss.hpp"
#include "./imu.hpp"
#include "tobas_setup_assistant/robot_info.hpp"

namespace gui
{
namespace sa
{
class ObserverWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = ObserverWidget;
  using super = BaseSettingWidget;

public:
  explicit ObserverWidget(
    const RobotInfo& robot,
    const ImuWidget* imu,
    const BarometerWidget* baro,
    const GnssWidget* gnss);

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void onOpened() override;
  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  YAML::Node staticParams() const;

private:
  const RobotInfo& robot_;
  const ImuWidget* imu_;
  const BarometerWidget* baro_;
  const GnssWidget* gnss_;

  QCheckBox* adaptive_gnss_noise_;
  QCheckBox* adaptive_grav_noise_;
  QCheckBox* do_acc_bias_estimation_;
  QCheckBox* do_gyro_bias_estimation_;
  QCheckBox* do_mag_hard_bias_estimation_;
  QCheckBox* do_mag_soft_bias_estimation_;
  QCheckBox* do_grav_estimation_;
};
};  // namespace sa
}  // namespace gui
