// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_urdf/exporter.hpp"

#include <urdf_parser/urdf_parser.h>

namespace tobas::urdf
{
tinyxml2::XMLDocument* exportUrdf(const ::urdf::ModelInterface& _model)
{
  // FIXME: Avoid deprecated function
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  return ::urdf::exportURDF(_model);
#pragma GCC diagnostic pop
}
}  // namespace tobas::urdf
