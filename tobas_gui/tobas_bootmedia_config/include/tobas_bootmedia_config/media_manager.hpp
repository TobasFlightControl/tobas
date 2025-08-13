#pragma once

#include <filesystem>

#include <QTimer>

#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/toggle_button.hpp>

#include "./bootmedia.hpp"

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

Q_SIGNALS:
  void connected(const Bootmedia& media);
  void disconnected();

public:
  explicit MediaManagerWidget();

private:
  QMap<QString, Bootmedia> medias_;

  qt::ComboBox* media_name_;
  qt::ToggleButton* connect_btn_;

  QTimer scan_timer_;

  static std::pair<std::string, std::string> getVendorAndModel(udev_device* dev);

private Q_SLOTS:
  void onConnectRequested();
  void onDisconnectRequested();
  void onScanTimerTimeout();
};
}  // namespace bm
}  // namespace gui
