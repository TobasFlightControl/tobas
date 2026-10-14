// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <filesystem>
#include <string>

#include <libudev.h>

namespace tobas
{
namespace udv
{
std::string getDevNode(udev_device* _dev);
std::string getPropertyValue(udev_device* _dev, const char* _key);
std::string getSysAttrValue(udev_device* _dev, const char* _attr);
std::string getBlockLabel(udev* _u, const std::filesystem::path& _devnode);
}  // namespace udv
}  // namespace tobas
