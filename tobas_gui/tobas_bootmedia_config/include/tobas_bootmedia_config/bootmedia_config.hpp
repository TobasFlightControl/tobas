#pragma once

#include <tobas_qt_tools/widgets/vertical_tab_widget.hpp>

#include "./hostname/hostname.hpp"
#include "./login_password/login_password.hpp"
#include "./media_manager.hpp"
#include "./ssh_authorized_keys/ssh_authorized_keys.hpp"
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

protected:
  void closeEvent(QCloseEvent* event) override;

private:
  MediaManagerWidget* media_manager_;
  tobas::qt::VerticalTabWidget* tabs_;

  HostnameWidget* hostname_;
  LoginPasswordWidget* login_password_;
  SshAuthorizedKeysWidget* ssh_keys_;
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
