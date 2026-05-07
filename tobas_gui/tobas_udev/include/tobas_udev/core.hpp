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
std::string getDevNode(udev_device* dev);
std::string getPropertyValue(udev_device* dev, const char* key);
std::string getSysAttrValue(udev_device* dev, const char* attr);
std::string getBlockLabel(udev* u, const std::filesystem::path& devnode);
}  // namespace udv
}  // namespace tobas
