// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tinyxml2.h>
#include <urdf_model/model.h>

namespace tobas
{
namespace urdf
{
tinyxml2::XMLDocument* exportUrdf(const ::urdf::ModelInterface& model);
}  // namespace urdf
}  // namespace tobas
