// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_udev/core.hpp"

namespace fs = std::filesystem;

namespace tobas::udv
{
namespace
{
std::string sanitize(const char* _s)
{
  if (!_s) {
    return {};
  }

  std::string out(_s);
  for (char& c : out) {
    if (c == '\n' || c == '\t') {
      c = ' ';
    }
  }

  return out;
}
}  // namespace

std::string getDevNode(udev_device* _dev)
{
  const auto devnode = udev_device_get_devnode(_dev);
  if (!devnode) {
    return {};
  }
  return std::string(devnode);
}

std::string getPropertyValue(udev_device* _dev, const char* _key)
{
  const auto value = udev_device_get_property_value(_dev, _key);
  if (!value) {
    return {};
  }
  return std::string(value);
}

std::string getSysAttrValue(udev_device* _dev, const char* _attr)
{
  const auto value = udev_device_get_sysattr_value(_dev, _attr);
  if (!value) {
    return {};
  }
  return sanitize(value);
}

std::string getBlockLabel(udev* _u, const fs::path& _devnode)
{
  const auto sysname = _devnode.filename().c_str();
  const auto dev = udev_device_new_from_subsystem_sysname(_u, "block", sysname);
  if (!dev) {
    return {};
  }
  return getPropertyValue(dev, "ID_FS_LABEL");
}
}  // namespace tobas::udv
