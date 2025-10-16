#include "tobas_simulation_gui/simulation_settings/sbus.hpp"

#include <QFormLayout>
#include <QVBoxLayout>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace gui
{
namespace sim
{
SbusWidget::SbusWidget() : dir_(kDirPath)
{
  device_names_ = new qt::ComboBox();

  // Layout
  const auto form = new QFormLayout();
  form->addRow("Device", device_names_);

  const auto rows = new QVBoxLayout();
  rows->addWidget(new qt::Label("S.BUS", cmn::kLabelPSize, QFont::Bold));
  rows->addLayout(form);

  setLayout(rows);

  // Connection
  connect(&scan_timer_, &QTimer::timeout, this, &self::onScanTimerTimeout);

  scan_timer_.start(1s);
}

fs::path SbusWidget::devicePath() const
{
  if (device_names_->count() == 0) {
    return {};
  }

  const auto device_name = device_names_->currentText().toStdString();
  return fs::path(kDirPath) / device_name;
}

void SbusWidget::onScanTimerTimeout()
{
  if (!dir_.exists()) {
    device_names_->clear();
    return;
  }

  const QSignalBlocker block(device_names_);

  // シンボリックリンク名を列挙
  QStringList filenames;
  for (const auto& entry : dir_.entryInfoList(QDir::NoDotAndDotDot | QDir::Files | QDir::System)) {
    filenames.append(entry.fileName());
  }
  filenames.sort(Qt::CaseInsensitive);

  // 既存選択を保存
  const auto cur_text = device_names_->currentText();

  // 選択肢を更新
  device_names_->clear();
  device_names_->addItem("");  // S.BUSドライバを立ち上げない選択肢を与える
  device_names_->addItems(filenames);

  // 以前の選択がまだ存在すれば復元
  const auto idx = device_names_->findText(cur_text);
  if (idx >= 0) {
    device_names_->setCurrentIndex(idx);
  }
}
}  // namespace sim
}  // namespace gui
