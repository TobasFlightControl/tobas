#pragma once

#include <filesystem>

#include <QComboBox>
#include <QDir>
#include <QTimer>
#include <QWidget>

#include <tobas_qt_tools/widgets/combo_box.hpp>

namespace tobas
{
namespace gui
{
namespace sim
{
class SbusWidget : public QWidget
{
  Q_OBJECT

  using self = SbusWidget;
  using super = QWidget;

  static constexpr char kDirPath[] = "/dev/serial/by-id";

public:
  explicit SbusWidget();

  std::filesystem::path devicePath() const;

private:
  const QDir dir_;

  qt::ComboBox* device_names_;

  QTimer scan_timer_;

private Q_SLOTS:
  void onScanTimerTimeout();
};
}  // namespace sim
}  // namespace gui
}  // namespace tobas
