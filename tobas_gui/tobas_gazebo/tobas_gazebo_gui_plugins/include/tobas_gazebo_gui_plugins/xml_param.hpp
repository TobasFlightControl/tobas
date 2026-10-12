// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tinyxml2.h>

namespace tobas
{
namespace gazebo
{
bool getXmlParam(const tinyxml2::XMLElement* _elem, const char* _name, double& _param);
}  // namespace gazebo
}  // namespace tobas
