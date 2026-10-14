// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tinyxml2.h>

#include "./model.hpp"

namespace tobas::uadf
{
tinyxml2::XMLDocument* exportUADF(const Model& _model);
}  // namespace tobas::uadf
