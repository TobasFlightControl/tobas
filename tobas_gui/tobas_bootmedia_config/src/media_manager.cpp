// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_bootmedia_config/media_manager.hpp"

#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <QDebug>
#include <QHBoxLayout>

#include <tobas_linux/core.hpp>
#include <tobas_linux/error.hpp>
#include <tobas_qt_tools/message.hpp>

#include "tobas_bootmedia_config/constants.hpp"

using namespace std::chrono_literals;

namespace tobas
{
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
  connect(connect_btn_, &qt::ToggleButton::checked, this, &self::onConnectRequested);
  connect(connect_btn_, &qt::ToggleButton::unchecked, this, &self::onDisconnectRequested);
  connect(&scan_timer_, &QTimer::timeout, this, &self::onScanTimerTimeout);

  scan_timer_.start(1s);
}

bool MediaManagerWidget::isConnected() const
{
  return connect_btn_->isChecked();
}

const BootMedia& MediaManagerWidget::currentBootMedia() const
{
  const auto media_name = media_name_->currentText();
  if (media_name.isEmpty()) {
    throw std::runtime_error("Boot device is not selected.");
  }

  return medias_.at(media_name);
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

  // 有効なメディアを探索
  std::unordered_map<QString, BootMedia> new_medias;
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

    // メディアを追加
    const BootMedia media(vendor.c_str(), model.c_str(), devnode.c_str());
    new_medias[media.string()] = media;
  }

  // udevを開放
  udev_enumerate_unref(en);
  udev_unref(udev_ctx);

  // 存在しないメディアをまとめる (ループ内で削除するとイテレータが狂うため)
  QSet<QString> removed_medias;
  for (const auto& [cur_media_name, _] : medias_) {
    if (!new_medias.contains(cur_media_name)) {
      removed_medias.insert(cur_media_name);
    }
  }

  // 存在しないメディアを選択肢から削除
  for (const auto& removed_media : removed_medias) {
    qInfo().noquote() << "Remove:" << removed_media;

    // 接続中に切断された場合
    if (isConnected() && media_name_->currentText() == removed_media) {
      qt::qErrorBox(this, "The connected media was ejected unexpectedly.");
      media_name_->setEnabled(true);
      connect_btn_->setChecked(false);
      Q_EMIT disconnected();
    }

    medias_.erase(removed_media);
    media_name_->removeText(removed_media);
  }

  // 新たなメディアを選択肢に追加
  for (const auto& [new_media_name, new_media] : new_medias) {
    if (!medias_.contains(new_media_name)) {
      qInfo().noquote() << "Add:" << new_media_name;

      medias_[new_media_name] = new_media;
      media_name_->addItem(new_media_name);
    }
  }

  // 未接続時は有効なメディアが存在する場合に限りConnectボタンを有効化
  if (!isConnected()) {
    connect_btn_->setEnabled(!medias_.empty());
  }
}

void MediaManagerWidget::onConnectRequested()
{
  // 管理者権限を確認
  if (!linux::isSuperUser()) {
    qt::qErrorBox(this, "Permission denied. Run as root (or use sudo) to perform this operation.");
    connect_btn_->setChecked(false);
    return;
  }

  // マウント先のディレクトリを作成
  if (mkdir(kBootPath, kPermission) < 0 && errno != EEXIST) {
    qt::qErrorBox(this, "Failed to create " + QString(kBootPath) + ".");
    connect_btn_->setChecked(false);
    return;
  }
  if (mkdir(kRootPath, kPermission) < 0 && errno != EEXIST) {
    qt::qErrorBox(this, "Failed to create " + QString(kRootPath) + ".");
    connect_btn_->setChecked(false);
    return;
  }

  // デバイスのパスを取得
  const auto& media = currentBootMedia();
  const auto sdx1 = media.devnode + '1';
  const auto sdx2 = media.devnode + '2';

  // 自動マウントされていれば先に外しておく
  cmd_exec_.execute("udisksctl unmount -b " + sdx1.toStdString() + " || true");
  cmd_exec_.execute("udisksctl unmount -b " + sdx2.toStdString() + " || true");

  // 外部ストレージをマウント
  if (mount(sdx1.toUtf8().constData(), kBootPath, "vfat", MS_NOATIME, nullptr) < 0) {
    qt::qErrorBox(this, "Failed to mount " + sdx1 + " on " + kBootPath + ": " + linux::strError().c_str());
    connect_btn_->setChecked(false);
    return;
  }
  if (mount(sdx2.toUtf8().constData(), kRootPath, "ext4", MS_NOATIME, nullptr) < 0) {
    qt::qErrorBox(this, "Failed to mount " + sdx2 + " on " + kRootPath + ": " + linux::strError().c_str());
    connect_btn_->setChecked(false);
    return;
  }

  // TODO: TobasOSであることを確認 (バージョンチェックも)

  // マウント中はメディア名を変更できないようにする
  media_name_->setEnabled(false);

  Q_EMIT connected(media);

  qt::qInfoBox(this, "The boot device was connected successfully.");
}

void MediaManagerWidget::onDisconnectRequested()
{
  // カーネルの書き込みキャッシュをストレージに吐き出す
  sync();

  // 外部ストレージをアンマウント
  if (umount2(kBootPath, 0) < 0) {
    qt::qErrorBox(this, "Failed to unmount " + QString(kBootPath) + ": " + linux::strError().c_str());
    connect_btn_->setChecked(true);
    return;
  }
  if (umount2(kRootPath, 0) < 0) {
    qt::qErrorBox(this, "Failed to unmount " + QString(kRootPath) + ": " + linux::strError().c_str());
    connect_btn_->setChecked(true);
    return;
  }

  // デバイス全体を安全に取り外す
  const auto& media = currentBootMedia();
  if (!cmd_exec_.execute("udisksctl power-off -b " + media.devnode.toStdString())) {
    qWarning().noquote().nospace() << "Failed to eject " << media.devnode << ": " << cmd_exec_.getOutput().c_str();
  }

  // 再びメディア名を選択可能にする
  media_name_->setEnabled(true);

  Q_EMIT disconnected();

  qt::qInfoBox(this, "The boot device was disconnected successfully.");
}
}  // namespace bm
}  // namespace gui
}  // namespace tobas
