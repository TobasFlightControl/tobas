#include "tobas_bootmedia_config/media_manager.hpp"

#include <QDebug>
#include <QHBoxLayout>

#include <tobas_udev/core.hpp>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace gui
{
namespace bm
{
MediaManagerWidget::MediaManagerWidget()
{
  media_name_ = new qt::ComboBox();
  media_name_->setFixedWidth(kMediaNameWidth);

  connect_btn_ = new qt::ToggleButton("Connect", "Disconnect");
  connect_btn_->setFixedWidth(kConnectButtonWidth);
  connect_btn_->setEnabled(false);

  // Layout
  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(media_name_);
  cols->addWidget(connect_btn_);

  // Connection
  connect(&scan_timer_, &QTimer::timeout, this, &self::onScanTimerTimeout);
  connect(connect_btn_, &qt::ToggleButton::checked, this, &self::onConnectRequested);
  connect(connect_btn_, &qt::ToggleButton::unchecked, this, &self::onDisconnectRequested);

  scan_timer_.setInterval(1s);
  scan_timer_.start();
}

std::pair<std::string, std::string> MediaManagerWidget::getVendorAndModel(udev_device* dev)
{
  auto vendor = udv::getPropertyValue(dev, "ID_VENDOR");
  auto model = udv::getPropertyValue(dev, "ID_MODEL");

  // 取得できなかった場合はUSBデバイスから補完
  if (vendor.empty() || model.empty()) {
    const auto usb = udev_device_get_parent_with_subsystem_devtype(dev, "usb", "usb_device");
    if (!usb) {
      return {};
    }

    if (vendor.empty()) {
      vendor = udv::getSysAttrValue(usb, "manufacturer");
      if (vendor.empty()) {
        return {};
      }
    }
    if (model.empty()) {
      model = udv::getSysAttrValue(usb, "product");
      if (model.empty()) {
        return {};
      }
    }
  }

  return { vendor, model };
}

void MediaManagerWidget::onScanTimerTimeout()
{
  const auto udev_ctx = udev_new();
  if (!udev_ctx) {
    qWarning() << "udev init failed.";
    return;
  }

  const auto en = udev_enumerate_new(udev_ctx);
  udev_enumerate_add_match_subsystem(en, "block");
  udev_enumerate_add_match_property(en, "DEVTYPE", "disk");
  udev_enumerate_add_match_property(en, "ID_DRIVE_REMOVABLE", "1");  // 内蔵ディスクを除外

  udev_enumerate_scan_devices(en);
  const auto devs = udev_enumerate_get_list_entry(en);

  for (auto it = devs; it; it = udev_list_entry_get_next(it)) {
    const auto syspath = udev_list_entry_get_name(it);
    const auto disk = udev_device_new_from_syspath(udev_ctx, syspath);
    if (!disk) {
      continue;
    }

    const auto devnode = udv::getDevNode(disk);  // e.g. /dev/sda
    if (devnode.empty()) {
      udev_device_unref(disk);
      continue;
    }

    // ラベルがbootfs/rootfsであることを確認
    const auto label1 = udv::getBlockLabel(udev_ctx, devnode + '1');
    const auto label2 = udv::getBlockLabel(udev_ctx, devnode + '2');
    if (label1 != "bootfs" || label2 != "rootfs") {
      udev_device_unref(disk);
      continue;
    }

    // ベンダとモデルを取得
    const auto [vendor, model] = getVendorAndModel(disk);
    if (vendor.empty() || model.empty()) {
      udev_device_unref(disk);
      continue;
    }

    // デバイスを開放
    udev_device_unref(disk);

    // TODO
  }

  udev_enumerate_unref(en);
  udev_unref(udev_ctx);
}

void MediaManagerWidget::onConnectRequested()
{
  // TODO
}

void MediaManagerWidget::onDisconnectRequested()
{
  // TODO
}
}  // namespace bm
}  // namespace gui
