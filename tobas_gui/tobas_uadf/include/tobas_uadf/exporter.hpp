// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <memory>

#include <tinyxml2.h>

#include "./model.hpp"

namespace tobas::uadf
{
std::unique_ptr<tinyxml2::XMLDocument> exportUADF(const Model& _model);
}  // namespace tobas::uadf
