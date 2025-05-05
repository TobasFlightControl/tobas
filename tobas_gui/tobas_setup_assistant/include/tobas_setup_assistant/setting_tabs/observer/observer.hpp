#pragma once

#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>
#include <tobas_qt_tools/widgets/stacked_widget.hpp>

#include "../barometer.hpp"
#include "../base_setting.hpp"
#include "../gnss.hpp"
#include "../imu.hpp"
#include "./base.hpp"
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

  static constexpr char kTypeKey[] = "observer_type";

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

  QString observerPackage() const;
  QString pluginName() const;

  YAML::Node staticParams() const;

private Q_SLOTS:
  void setCurrentObserver(int index);

private:
  const RobotInfo& robot_;
  const ImuWidget* imu_;
  const BarometerWidget* baro_;
  const GnssWidget* gnss_;

  qt::ComboBox* type_;
  qt::StackedWidget* observers_;
  qt::DescriptionWidget* description_;

  BaseObserverWidget* selected();
  const BaseObserverWidget* selected() const;
};
};  // namespace sa
}  // namespace gui
