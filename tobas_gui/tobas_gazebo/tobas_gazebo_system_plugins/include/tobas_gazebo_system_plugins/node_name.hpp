// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>

namespace tobas
{
namespace gazebo
{
/* 文字列をROSノード名に使用可能なものに修正する． */
std::string sanitizeNodeName(std::string str);
}  // namespace gazebo
}  // namespace tobas
