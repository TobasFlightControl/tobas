// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>

#include <tobas_udev/core.hpp>

int main()
{
  const auto udev_ctx = udev_new();
  if (!udev_ctx) {
    std::cerr << "udev init failed." << std::endl;
    return EXIT_FAILURE;
  }

  const auto en = udev_enumerate_new(udev_ctx);
  udev_enumerate_add_match_subsystem(en, "block");
  udev_enumerate_add_match_property(en, "DEVTYPE", "disk");
  udev_enumerate_add_match_property(en, "ID_DRIVE_REMOVABLE", "1");  // Exclude internal disks.

  udev_enumerate_scan_devices(en);
  const auto devs = udev_enumerate_get_list_entry(en);

  for (auto it = devs; it; it = udev_list_entry_get_next(it)) {
    const auto syspath = udev_list_entry_get_name(it);
    const auto disk = udev_device_new_from_syspath(udev_ctx, syspath);
    if (!disk) {
      continue;
    }

    const auto devnode = tobas::udv::getDevNode(disk);  // e.g. /dev/sda
    if (devnode.empty()) {
      udev_device_unref(disk);
      continue;
    }

    // Get media information.
    const auto vendor = tobas::udv::getPropertyValue(disk, "ID_VENDOR");
    const auto model = tobas::udv::getPropertyValue(disk, "ID_MODEL");
    const auto label1 = tobas::udv::getBlockLabel(udev_ctx, devnode + '1');
    const auto label2 = tobas::udv::getBlockLabel(udev_ctx, devnode + '2');
    if (vendor.empty() || model.empty() || label1.empty() || label2.empty()) {
      udev_device_unref(disk);
      continue;
    }

    // Release the device.
    udev_device_unref(disk);

    std::cout << vendor << " " << model << " (" << label1 << ", " << label2 << ") (" << devnode << ")" << std::endl;
  }

  udev_enumerate_unref(en);
  udev_unref(udev_ctx);
}
