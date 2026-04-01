// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>

#include <tinyxml2.h>

namespace tobas
{
namespace xml
{
std::string xmlDocumentToString(const tinyxml2::XMLDocument* doc);
}  // namespace xml
}  // namespace tobas
