#pragma once

#include <tobas_qt_tools/widgets/vertical_tab_widget.hpp>

#include "./wifi_sta.hpp"

namespace gui
{
namespace bm
{
class BootmediaConfigWidget : public QWidget
{
  Q_OBJECT

  using self = BootmediaConfigWidget;
  using super = QWidget;

  static constexpr int kTabHeight = 35;  // これ以上無いと何故かTabBarの文字が横に見切れてしまう
  static constexpr int kTabWidth = 70;

public:
  explicit BootmediaConfigWidget();

  void reset();

private:
  qt::VerticalTabWidget* tabs_;

  WifiStationWidget* wifi_sta_;

  void setTabsEnabled(bool enabled);
};
}  // namespace bm
}  // namespace gui
