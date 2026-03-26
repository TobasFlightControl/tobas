#pragma once

#include <QTimer>

#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/toggle_button.hpp>
#include <tobas_udev/core.hpp>

#include "./bootmedia.hpp"

namespace tobas
{
namespace gui
{
namespace bm
{
class MediaManagerWidget : public QWidget
{
  Q_OBJECT

  using self = MediaManagerWidget;
  using super = QWidget;

  static constexpr int kMediaNameWidth = 600;
  static constexpr int kConnectButtonWidth = 100;

  static constexpr mode_t kPermission = 0755;  // rwxr-xr-x

Q_SIGNALS:
  void connected(const BootMedia& media);
  void disconnected();

public:
  explicit MediaManagerWidget();

  bool isConnected() const;

private:
  std::unordered_map<QString, BootMedia> medias_;

  qt::ComboBox* media_name_;
  qt::ToggleButton* connect_btn_;

  QTimer scan_timer_;

  const BootMedia& currentBootMedia() const;

  static std::pair<std::string, std::string> getVendorAndModel(udev_device* dev);

private Q_SLOTS:
  void onConnectRequested();
  void onDisconnectRequested();
  void onScanTimerTimeout();
};
}  // namespace bm
}  // namespace gui
}  // namespace tobas
