#include <iostream>

#include <tobas_udev/core.hpp>

using namespace std;
namespace fs = filesystem;

int main()
{
  const auto udev_ctx = udev_new();
  if (!udev_ctx) {
    cerr << "udev init failed." << endl;
    return EXIT_FAILURE;
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

    // メディアの情報を取得
    const auto vendor = udv::getPropertyValue(disk, "ID_VENDOR");
    const auto model = udv::getPropertyValue(disk, "ID_MODEL");
    const auto label1 = udv::getBlockLabel(udev_ctx, devnode + '1');
    const auto label2 = udv::getBlockLabel(udev_ctx, devnode + '2');
    if (vendor.empty() || model.empty() || label1.empty() || label2.empty()) {
      udev_device_unref(disk);
      continue;
    }

    // デバイスを開放
    udev_device_unref(disk);

    cout << vendor << " " << model << " (" << label1 << ", " << label2 << ") (" << devnode << ")" << endl;
  }

  udev_enumerate_unref(en);
  udev_unref(udev_ctx);
}
