#pragma once

#include <QCheckBox>

#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>
#include <tobas_qt_tools/widgets/stacked_widget.hpp>

#include "./base_setting.hpp"

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
  explicit ObserverWidget();

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void onOpened() override;
  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  bool adaptiveGnssNoise() const;
  bool adaptiveGravityNoise() const;
  bool doAccelBiasEstimation() const;
  bool doGyroBiasEstimation() const;
  bool doMagHardBiasEstimation() const;
  bool doMagSoftBiasEstimation() const;
  bool doGravityEstimation() const;

private:
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
