// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tinyxml2.h>

namespace tobas
{
namespace gazebo
{
bool getXmlParam(const tinyxml2::XMLElement* elem, const char* name, double& param);
}  // namespace gazebo
}  // namespace tobas
