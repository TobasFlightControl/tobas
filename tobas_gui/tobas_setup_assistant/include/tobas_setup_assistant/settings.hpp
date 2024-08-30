#pragma once

#include <tobas_qt_tools/widgets/vertical_tab_widget.hpp>

#include "./setting_tabs/battery/battery.hpp"
// TODO

namespace gui
{
namespace setup_assistant
{
class SettingsWidget : public qt::VerticalTabWidget
{
  Q_OBJECT

  using self = SettingsWidget;
  using super = qt::VerticalTabWidget;

  static constexpr int kTabHeight = 30;  // 30以上無いと何故かTabBarの文字が横に見切れてしまう
  static constexpr int kTabWidth = 70;
  static constexpr int kSettingsMinHeight = 300;

public:
  explicit SettingsWidget();

  void updateInternalDataStructures();

  YAML::Node dump();
  bool load(const YAML::Node& node);

private:
  BatteryWidget* battery_;
  // TODO
};
}  // namespace setup_assistant
}  // namespace gui
