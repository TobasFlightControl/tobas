// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <memory>

#include <tinyxml2.h>
#include <urdf_model/model.h>

namespace tobas::urdf
{
/** Export as URDF 1.2 with numeric formatting matching the legacy `urdf::exportURDF`. */
std::unique_ptr<tinyxml2::XMLDocument> exportUrdf(const ::urdf::ModelInterface& _model);
}  // namespace tobas::urdf
