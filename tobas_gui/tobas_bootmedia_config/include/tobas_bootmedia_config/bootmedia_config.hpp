#pragma once

#include <tobas_qt_tools/widgets/vertical_tab_widget.hpp>

#include "./hostname/hostname.hpp"
#include "./login_password/login_password.hpp"
#include "./media_manager.hpp"
#include "./wifi_client/wifi_client.hpp"
#include "./wifi_hotspot/wifi_hotspot.hpp"

namespace tobas
{
namespace gui
{
namespace bm
{
class BootmediaConfigWidget : public QWidget
{
  Q_OBJECT

  using self = BootmediaConfigWidget;
  using super = QWidget;

  static constexpr int kTabHeight = 35;
  static constexpr int kTabWidth = 70;

public:
  explicit BootmediaConfigWidget();

  void reset();

private:
  MediaManagerWidget* media_manager_;
  qt::VerticalTabWidget* tabs_;

  HostnameWidget* hostname_;
  LoginPasswordWidget* login_password_;
  WifiClientWidget* wifi_client_;
  WifiHotspotWidget* wifi_hotspot_;

  void setTabsEnabled(bool enabled);

private Q_SLOTS:
  void onMediaConnected();
  void onMediaDisconnected();
};
}  // namespace bm
}  // namespace gui
}  // namespace tobas
